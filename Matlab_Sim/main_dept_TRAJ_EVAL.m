%% LOCALIZATION PROBLEM - UNICYCLE, RANGE-ONLY EKF
% TRAJECTORY EVALUATION TEST - trace a rectilinear target trajectory
% between two far apart anchors, follow in real time test, linear
% regression of estimated positions to evaluate std, var from ideal traj.
clearvars; clc; close all;

%% Variables
tracking_node_ids = [108, 113:119, 121:154];
default_noise = 0.07;
z_fixed_m = 1.50; % [m] tag height from the ceiling

% Room-constraint flags (hooks kept explicit for later pipeline steps).
enable_room_constraint = true;
room_model_path = "room_constraint_model_projective.mat";
room_constraint_on_init = false;
room_constraint_on_predict = true;
room_constraint_on_update = true;
room_projection_threshold_m = 0.85;

% Placeholder flag for optional distance-transform logic.
enable_distance_penalty = false;

%% Serial reader
baud_rate = 115200;

if ismac || isunix
    % Note: On Unix/Mac, ports are typically in the /dev/ directory.
    serial_port = "/dev/tty.usbmodem00000000050C1";
elseif ispc
    serial_port = "COM4";
else
    error('Platform not supported');
end

%% Map Loading
DEPTevb1000map = importfile("DEPT_evb1000_map.csv");
map = DEPTevb1000map(ismember(DEPTevb1000map.NodeId, tracking_node_ids), :);
node_coords = split(map.Coordinates, ",");
map.lat = str2double(regexprep(node_coords(:,1), '[\[\]]', ''));
map.lon = str2double(regexprep(node_coords(:,2), '[\[\]]', ''));
if size(node_coords, 2) >= 3
    map.z = str2double(regexprep(node_coords(:,3), '[\[\]]', ''));
else
    map.z = zeros(height(map), 1);
end
addr_short = lower(extractAfter(map.evb1000, strlength(map.evb1000) - 5));

%% Load room occupancy constraint
room_constraint = [];
if enable_room_constraint
    if isfile(room_model_path)
        room_data = load(room_model_path, 'room');
        room_constraint = room_data.room;
        grid_size = size(room_constraint.grid.free_mask);
        fprintf("[init] room model loaded %dx%d\n", grid_size(2), grid_size(1));
        fprintf("[init] Room constraint hooks: init=%d predict=%d update=%d\n", ...
            room_constraint_on_init, room_constraint_on_predict, room_constraint_on_update);
    else
        fprintf("[init] %s not found, running without occupancy constraint\n", room_model_path);
    end
else
    fprintf("[init] Room occupancy constraint disabled by flag\n");
end

if enable_distance_penalty
    fprintf("[init] Distance-transform penalty flag is ON, but the feature is not implemented yet\n");
else
    fprintf("[init] Distance-transform penalty disabled by flag\n");
end

%% Load per-anchor variances
if isfile("variance_per_node_dept.csv")
    var_table = readtable("variance_per_node_dept.csv");
    noise_map = containers.Map(var_table.node_id, var_table.pooled_std_m);
    bias_map = containers.Map('KeyType', 'double', 'ValueType', 'double');
    BIAS_THRESHOLD_M = 2; % ignore anchor biases larger than 50 cm
    bad_bias_nodes = [];
    for row_idx = 1:height(var_table)
        node_id = double(var_table.node_id(row_idx));
        if ismember(node_id, bad_bias_nodes) || isnan(var_table.mean_bias_mm(row_idx))
            continue;
        end
        bias_m = var_table.mean_bias_mm(row_idx) / 1000.0;
        if abs(bias_m) <= BIAS_THRESHOLD_M
            bias_map(node_id) = bias_m;
        end
    end
    fprintf("[init] variance map loaded: %d entries\n", height(var_table));
else
    noise_map = containers.Map('KeyType', 'double', 'ValueType', 'double');
    bias_map = containers.Map('KeyType', 'double', 'ValueType', 'double');
    fprintf("[init] variance_per_node_dept.csv not found, default use %.3f m\n", default_noise);
end

%% System definition (Unicycle EKF)
% State definition:
% x = [px, py, theta, v, omega]^T
%   px, py  : position [m]
%   theta   : heading [rad]
%   v       : linear speed [m/s]
%   omega   : yaw rate [rad/s]

% Sampling time.
dT = 0.5;

% Process-noise hyperparameters (on speed and yaw-rate channels).
sigma_v_process = 0.12;       % [m/s]
sigma_omega_process = 0.08;   % [rad/s]

% Unicycle dynamics x_{k+1} = f(x_k).
fun = @(xk) [ ...
    xk(1) + xk(4)*cos(xk(3))*dT; ...
    xk(2) + xk(4)*sin(xk(3))*dT; ...
    xk(3) + xk(5)*dT; ...
    xk(4); ...
    xk(5)];

% Jacobian A_k = df/dx evaluated at x_k.
A_unicycle = @(xk) [ ...
    1, 0, -xk(4)*sin(xk(3))*dT,  cos(xk(3))*dT, 0; ...
    0, 1,  xk(4)*cos(xk(3))*dT,  sin(xk(3))*dT, 0; ...
    0, 0, 1,                      0,             dT; ...
    0, 0, 0,                      1,             0; ...
    0, 0, 0,                      0,             1];

% Noise injection matrix G_k and input-noise covariance Q_u:
%   v_{k+1} = v_k + eps_v
%   w_{k+1} = w_k + eps_w
G_unicycle = [ ...
    0, 0; ...
    0, 0; ...
    0, 0; ...
    1, 0; ...
    0, 1];

Q_u = diag([sigma_v_process^2, sigma_omega_process^2]);
Q_unicycle = @(~) (G_unicycle * Q_u * G_unicycle.');

% Range measurement model (with fixed tag height):
% z_i = sqrt((px-ax_i)^2 + (py-ay_i)^2 + (z_fixed-az_i)^2) + r_i
h_range = @(xk, anchors_xyz) sqrt((xk(1) - anchors_xyz(:,1)).^2 + ...
                                  (xk(2) - anchors_xyz(:,2)).^2 + ...
                                  (z_fixed_m - anchors_xyz(:,3)).^2);

% Measurement Jacobian H_k for the range model.
% Each row i: [ (px-ax_i)/ri, (py-ay_i)/ri, 0, 0, 0 ]
H_range = @(xk, anchors_xyz) build_range_jacobian_unicycle(xk, anchors_xyz, z_fixed_m);

%% Trilateration (WLS) + P0 initialization
cfg_init = struct();
cfg_init.mode = "multi";                          % "single" | "multi"
cfg_init.max_init_attempts = 45;                  % max serial rounds scanned during initialization
cfg_init.initial_heading_std_rad = deg2rad(75);   % std dev of initial heading uncertainty
cfg_init.initial_speed_std_mps = 0.70;            % std dev of initial linear speed
cfg_init.initial_yaw_rate_std_radps = 0.90;       % std dev of initial yaw rate
cfg_init.min_xy_variance_m2 = 0.20^2;             % minimum variance floor on x/y at initialization
cfg_init.range_variance_distance_gain = 2e-5;     % k in sigma^2(d) = sigma0^2 + k*d^2
cfg_init.bootstrap_target_rounds = 15;            % target count of valid WLS rounds used in multi-round init
cfg_init.bootstrap_min_valid_rounds = 4;          % minimum valid WLS rounds required in multi-round init
cfg_init.bootstrap_min_anchor_count = 3;          % minimum anchors in one round to accept it for bootstrap
cfg_init.bootstrap_mae_gate_k = 2.0;              % robust gate factor for round MAE outlier rejection
cfg_init.bootstrap_mae_gate_min_m = 0.20;         % minimum absolute MAE gate [m]
cfg_init.bootstrap_xy_gate_k = 2.5;               % robust gate factor for XY outlier rejection
cfg_init.bootstrap_xy_gate_min_m = 0.35;          % minimum absolute XY gate [m]
cfg_init.bootstrap_min_heading_span_m = 0.30;     % minimum displacement span to trust heading from bootstrap
cfg_init.bootstrap_heading_std_floor_rad = deg2rad(12); % lower bound on theta std when heading is estimated

% Shared corridor geometry (used by both init fallback-heading and runtime heading prior).
corridor_half_width_m = 1.40; % [m]
corridor_segments_xyxy = [ ...
    133.6, 26.1, 188.6, 26.1; ...
    133.6, 10.9, 133.6, 26.1; ...
    133.6, 10.9, 188.6, 10.9];
atrium_polygon_xy = [ ...
    181.0, 24.0; ...
    189.4, 24.0; ...
    189.4, 12.8; ...
    181.0, 12.8];
cfg_init.enable_corridor_heading_fallback = true;
cfg_init.corridor_half_width_m = corridor_half_width_m;
cfg_init.corridor_segments_xyxy = corridor_segments_xyxy;
cfg_init.disable_corridor_prior_in_atrium = true;
cfg_init.atrium_polygon_xy = atrium_polygon_xy;

% Runtime visualization settings.
viz_cfg = struct();
viz_cfg.heading_sigma_mult = 2.0;      % cone half-angle = n_sigma * std(theta)
viz_cfg.heading_cone_length_m = 1.4;   % cone radius in meters
viz_cfg.heading_cone_points = 40;      % arc resolution
viz_cfg.cov_ellipse_sigma_mult = 2.0;  % ellipse scale in sigma units
viz_cfg.cov_ellipse_points = 90;       % ellipse resolution

%% GIF recording setup
script_dir = fileparts(mfilename('fullpath'));
project_root = fileparts(script_dir);
gif_cfg = struct();
gif_cfg.enabled = true;
gif_cfg.delay_time_s = dT;
gif_cfg.capture_every_n = 1;
gif_cfg.output_basename = "main_dept_map_recording";
gif_cfg.output_dir = fullfile(project_root, 'recordings', 'dept');

%% TRAJ_EVAL rectilinear reference + logging setup
traj_eval_cfg = struct();
traj_eval_cfg.enabled = true;
traj_eval_cfg.logging_enabled = true;
% Default rectilinear test: long upper DEPT corridor, from node 123 to node 138.
% Change these two ids to evaluate a different straight segment.
traj_eval_cfg.rect_anchor_node_ids = [134, 126];
traj_eval_cfg.logs_root_dir = fullfile(project_root, 'TRAJ_EVAL_logs');
traj_eval_cfg.session_prefix = "dept_rectilinear";
traj_eval = init_traj_eval(traj_eval_cfg, map, dT);
if traj_eval.enabled
    fprintf('[traj_eval] reference nodes %d -> %d | length=%.2f m | heading=%.1f deg\n', ...
        traj_eval.reference.anchor_node_ids(1), traj_eval.reference.anchor_node_ids(2), ...
        traj_eval.reference.length_m, rad2deg(traj_eval.reference.heading_rad));
    if traj_eval.logging_enabled
        fprintf('[traj_eval] logging samples to %s\n', traj_eval.csv_path);
    end
end

fprintf('[init] opening serial %s @ %d\n', serial_port, baud_rate);
while true
    try
        s = serialport(serial_port, baud_rate);
        break;
    catch ME
        fprintf('[init] serial open failed: %s\n', ME.message);
        pause(2);
    end
end
configureTerminator(s, "LF");
flush(s);

%% Plot setup
% Max buffer size for position estimates (dots) and heading estimates
% (quivers)
max_trail_points = 80; % approx. 2 points per second ==> plot 40s of estimates
% Buffers for u and v (lateral and longitudinal components of heading
% vector)
traj_u = [];
traj_v = [];

all_anchors = [map.lat, map.lon];
map_fig = figure; hold on; grid on;
ax_main = gca;
plot(all_anchors(:,1), all_anchors(:,2), 'b^', 'MarkerSize', 8, 'DisplayName', 'Anchors');
line_est = plot(NaN, NaN, 'g-o', 'MarkerSize', 5, 'LineWidth', 1.5, 'DisplayName', 'Stima EKF');
dot_est = plot(NaN, NaN, 'r*', 'MarkerSize', 14, 'DisplayName', 'Posizione attuale');
quiv_est = quiver(NaN, NaN, NaN, NaN, 0, 'Color', 'g', 'LineWidth', 1.5, ...
    'MaxHeadSize', 0.5, 'DisplayName', 'Heading stimato');
h_used_anchors = scatter(NaN, NaN, 65, NaN, 'filled', ...
    'MarkerEdgeColor', [0.10, 0.10, 0.10], ...
    'DisplayName', 'Anchors usate (var)');
h_cov_ellipse = plot(NaN, NaN, 'm-', 'LineWidth', 1.6, 'DisplayName', 'Ellisse covarianza');
h_heading_cone = patch('XData', NaN, 'YData', NaN, ...
    'FaceColor', [0.15, 0.65, 0.15], 'FaceAlpha', 0.14, ...
    'EdgeColor', [0.05, 0.45, 0.05], 'LineStyle', '--', 'LineWidth', 1.1, ...
    'DisplayName', 'Cono incertezza heading');
h_traj_eval_closest = [];
h_traj_eval_error = [];
if traj_eval.enabled
    ref_xy = traj_eval.reference.points_xy;
    plot(ref_xy(:,1), ref_xy(:,2), 'k--', 'LineWidth', 2.0, ...
        'DisplayName', 'Traiettoria rettilinea target');
    plot(ref_xy(:,1), ref_xy(:,2), 'ks', 'MarkerSize', 8, 'LineWidth', 1.5, ...
        'MarkerFaceColor', [1.0, 0.90, 0.15], 'DisplayName', 'Estremi test rettilineo');
    h_traj_eval_closest = plot(NaN, NaN, 'ko', 'MarkerSize', 8, 'LineWidth', 1.3, ...
        'MarkerFaceColor', [1.0, 0.70, 0.10], 'DisplayName', 'Punto target piu vicino');
    h_traj_eval_error = plot(NaN, NaN, '-', 'Color', [0.90, 0.35, 0.05], ...
        'LineWidth', 1.2, 'DisplayName', 'Errore vs traiettoria');
end
colormap(ax_main, parula(256));
cb_anchor_var = colorbar(ax_main, 'eastoutside');
cb_anchor_var.Label.String = 'Varianza ancore usate [m^2]';
xlabel('X [m]'); ylabel('Y [m]');
title('Real-Time UWB EKF Tracking  |  adaptive initialization');
legend('Location', 'best');

x_span = max(all_anchors(:,1)) - min(all_anchors(:,1));
y_span = max(all_anchors(:,2)) - min(all_anchors(:,2));
status_text = text( ...
    min(all_anchors(:,1)) + 0.02 * max(x_span, 1.0), ...
    max(all_anchors(:,2)) - 0.05 * max(y_span, 1.0), ...
    'Waiting for first valid initialization round...', ...
    'Color', [0.35, 0.35, 0.35], ...
    'FontWeight', 'bold');
drawnow;
gif_recorder = map_gif_recorder('start', gif_cfg);
gif_recorder = map_gif_recorder('capture', gif_recorder, map_fig);

[x0, P0, init_info] = initialize_state_weighted( ...
    s, map, addr_short, noise_map, bias_map, default_noise, z_fixed_m, cfg_init, ...
    room_constraint, room_constraint_on_init, room_projection_threshold_m);

init_mode_char = char(string(init_info.mode));
init_heading_src_char = char(string(init_info.heading_source));
fprintf('[init] mode=%s accepted=%d/%d last_round=%d mae=%.2fm heading=%s\n', ...
    init_mode_char, init_info.accepted_rounds, init_info.total_attempts, ...
    init_info.round_id, init_info.mae_m, init_heading_src_char);
fprintf('[init] distance variance model: sigma^2(d)=sigma0^2 + %.2e*d^2\n', cfg_init.range_variance_distance_gain);
fprintf('[init] x0=[%.2f %.2f %.2f %.2f %.2f]\n', x0(1), x0(2), x0(3), x0(4), x0(5));
fprintf('[init] P0 diag=[%.4f %.4f %.4f %.4f %.4f]\n', P0(1,1), P0(2,2), P0(3,3), P0(4,4), P0(5,5));

set(status_text, 'String', sprintf('Init done | r=%d | MAE=%.2fm', init_info.round_id, init_info.mae_m), ...
    'Color', [0.00, 0.45, 0.00]);
set(line_est, 'XData', x0(1), 'YData', x0(2));
set(dot_est, 'XData', x0(1), 'YData', x0(2));
set(quiv_est, 'XData', x0(1), 'YData', x0(2), 'UData', 0.8*cos(x0(3)), 'VData', 0.8*sin(x0(3)));
[ell_x0, ell_y0] = covariance_ellipse_points(x0(1:2), P0(1:2,1:2), ...
    viz_cfg.cov_ellipse_sigma_mult, viz_cfg.cov_ellipse_points);
[cone_x0, cone_y0] = heading_cone_points(x0(1:2), x0(3), sqrt(max(P0(3,3), 0)), ...
    viz_cfg.heading_sigma_mult, viz_cfg.heading_cone_length_m, viz_cfg.heading_cone_points);
set(h_cov_ellipse, 'XData', ell_x0, 'YData', ell_y0);
set(h_heading_cone, 'XData', cone_x0, 'YData', cone_y0);
set(h_used_anchors, 'XData', NaN, 'YData', NaN, 'CData', NaN);
drawnow;
gif_recorder = map_gif_recorder('capture', gif_recorder, map_fig);

% Keep x0/P0 in workspace for next implementation steps (predict/update).
x_est = x0; 
P_est = P0;
x_prev_est = x_est; % Previous state estimate --> useful to infer theta (heading angle)
traj_eval = set_traj_eval_start_round(traj_eval, init_info.round_id);
init_rr_log = struct('round_id', init_info.round_id, 'raw_count', init_info.raw_count, ...
    'used_count', init_info.used_count);
[traj_eval, init_eval_row] = append_traj_eval_log( ...
    traj_eval, x_est, P_est, init_rr_log, 0, "init", string(init_info.mode), ...
    init_info.used_count, trace(P_est(1:2,1:2)));
update_traj_eval_plot_handles(init_eval_row, x_est, h_traj_eval_closest, h_traj_eval_error);
fprintf('[init] step-3 complete: weighted trilateration + P0 ready\n');

%% Main loop - EKF tracking (predict + update per round)
traj_x = x_est(1);
traj_y = x_est(2);
I5 = eye(5);
k_loop = 0;
last_round_id = init_info.round_id;
anchor_var_min_m2 = Inf;
anchor_var_max_m2 = -Inf;
dist_moved_thresh = 0.5; % [m]
sigma_theta = 0.5;
%  HEADING PRIOR CONFIG 
% We inject a pseudo-measurement on heading theta during prediction.
% Idea:
% - If we are NOT in a corridor: heading target = displacement direction.
% - If we are IN a corridor    : heading target = corridor axis direction.
% The confidence of this pseudo-measurement is tuned through R_theta:
% smaller R_theta => stronger correction on theta.
heading_prior_cfg = struct();
heading_prior_cfg.enabled = true;
% Minimum displacement needed to trust displacement-derived heading.
heading_prior_cfg.min_motion_for_heading_m = dist_moved_thresh;
% Std used outside corridors (larger => weaker heading prior).
heading_prior_cfg.base_heading_std_rad = sigma_theta;
% Std used inside corridors (smaller => stronger alignment to corridor).
heading_prior_cfg.corridor_heading_std_rad = deg2rad(10);
% Distance from corridor centerline under which corridor prior is active.
heading_prior_cfg.corridor_half_width_m = corridor_half_width_m;            % [m]
% If true: snap directly to axis heading.
% If false: blend motion heading and axis heading based on lateral distance.
heading_prior_cfg.use_axis_heading_only_in_corridor = true;
% Corridor centerlines as [x1 y1 x2 y2] segments (blue area).
heading_prior_cfg.corridor_segments_xyxy = corridor_segments_xyxy;
% Atrium polygon (green area): inside this region, do NOT apply corridor axis forcing (keep only motion-derived heading prior).
heading_prior_cfg.disable_corridor_prior_in_atrium = true;
heading_prior_cfg.atrium_polygon_xy = atrium_polygon_xy;

fprintf('[track] EKF loop started\n');

while true
    if ~isgraphics(map_fig) || ~isgraphics(line_est) || ~isgraphics(dot_est) || ~isgraphics(h_cov_ellipse)
        fprintf('[track] figure closed, stopping loop\n');
        break;
    end

    % Read next complete round with all currently available anchors.
    rr = collect_round_measurements_all(s, map, addr_short, noise_map, bias_map, default_noise, map_fig);
    if isempty(rr)
        fprintf('[track] figure closed while waiting for serial data, stopping loop\n');
        break;
    end

    % Skip duplicated round id if serial stream re-emits the same round.
    if ~isnan(rr.round_id) && rr.round_id == last_round_id
        continue;
    end
    last_round_id = rr.round_id;
    k_loop = k_loop + 1;

    % EKF Predict
    x_pred = fun(x_est);
    A_k = A_unicycle(x_est);
    P_pred = A_k * P_est * A_k.' + Q_unicycle(x_est);
    P_pred = 0.5 * (P_pred + P_pred.');

    if room_constraint_on_predict && ~isempty(room_constraint)
        [xy_pred_room, info_pred_room] = apply_room_constraint(room_constraint, x_pred(1:2).', ...
            'projection_threshold_m', room_projection_threshold_m, 'fallback_pose', x_est(1:2).', 'label', 'main_new_predict');
        if strcmp(info_pred_room.action, 'project')
            x_pred(1:2) = xy_pred_room(:);
            fprintf('[room] round %d predict projected (dist=%.2f m)\n', rr.round_id, info_pred_room.distance_to_free_m);
        elseif strcmp(info_pred_room.action, 'reject')
            x_pred(1:2) = xy_pred_room(:);
            x_pred(4) = 0;
            x_pred(5) = 0;
            fprintf('[room] round %d predict rejected by map -> fallback to previous estimate\n', rr.round_id);
        end
    end

    mode_str = "prediction";
    note_str = "nominal";
    used_count = rr.used_count;
    sigma2_used = range_variance_model(rr.std_m(:), rr.distances_m(:), cfg_init.range_variance_distance_gain);

    % HEADING PSEUDO-MEASUREMENT 
    % This is a Kalman update on theta only (same EKF step, before range update):
    % 1) build a heading target (motion-based or corridor-based),
    % 2) compute innovation on angle manifold (wrap to [-pi, pi]),
    % 3) apply scalar Kalman correction through H_theta=[0 0 1 0 0].
    [x_pred, P_pred, heading_prior_info] = apply_heading_prior_unicycle( ...
        x_pred, P_pred, x_prev_est, I5, heading_prior_cfg);
    % Keep trace in runtime log when corridor axis prior was used.
    if heading_prior_info.applied && heading_prior_info.used_corridor_axis
        note_str = note_str + "+head_corr";
    end

    % EKF Update (works for both single-anchor and multi-anchor rounds)
    if used_count >= 1 && ~isempty(rr.anchors_xyz) && size(rr.anchors_xyz, 1) == length(rr.distances_m)
        z_k = rr.distances_m(:);
        h_k = h_range(x_pred, rr.anchors_xyz);
        H_k = H_range(x_pred, rr.anchors_xyz);
        % CRITICAL CHECK: Ensure H_k dimensions match [Measurements x States]
        if size(H_k, 1) == length(z_k) && size(H_k, 2) == 5
            R_k = diag(sigma2_used);
            innov_k = z_k - h_k;
            S_k = H_k * P_pred * H_k.' + R_k;
            S_k = 0.5 * (S_k + S_k.');
        else
            used_count = 0;
        end

        if used_count == 1
            S_scalar = S_k(1,1);
            S_scalar = max(S_scalar, 1e-9);
            K_k = (P_pred * H_k.') / S_scalar;
        else
            if rcond(S_k) < 1e-10
                S_inv = pinv(S_k);
                K_k = P_pred * H_k.' * S_inv;
            else
                K_k = (P_pred * H_k.') / S_k;
            end
        end

        x_upd = x_pred + K_k * innov_k;
        P_upd = (I5 - K_k * H_k) * P_pred * (I5 - K_k * H_k).' + K_k * R_k * K_k.';
        mode_str = "update";
    else
        x_upd = x_pred;
        P_upd = P_pred;
    end

    if room_constraint_on_update && ~isempty(room_constraint)
        [xy_upd_room, info_upd_room] = apply_room_constraint(room_constraint, x_upd(1:2).', ...
            'projection_threshold_m', room_projection_threshold_m, 'fallback_pose', x_pred(1:2).', 'label', 'main_new_update');
        if strcmp(info_upd_room.action, 'project')
            x_upd(1:2) = xy_upd_room(:);
            P_upd(1:2,1:2) = P_upd(1:2,1:2) + (0.10^2) * eye(2);
            mode_str = "map project";
            note_str = "project";
            fprintf('[room] round %d update projected (dist=%.2f m, thr=%.2f m)\n', ...
                rr.round_id, info_upd_room.distance_to_free_m, room_projection_threshold_m);
        elseif strcmp(info_upd_room.action, 'reject')
            x_upd = x_est;
            P_upd = P_est;
            x_upd(4) = 0;
            x_upd(5) = 0;
            mode_str = "map reject";
            note_str = "reject";
            fprintf('[room] round %d update rejected (dist=%.2f m > thr=%.2f m), holding previous valid state\n', ...
                rr.round_id, info_upd_room.distance_to_free_m, room_projection_threshold_m);
        end
    end

    % Finalize state
    x_upd(3) = wrap_angle_pi(x_upd(3));
    P_upd = 0.5 * (P_upd + P_upd.');

    x_est = x_upd;
    P_est = P_upd;
    x_prev_est = x_est;

    % Plot update --> enforce FIFO behavior for traj_x, traj_y, traj_u,
    % traj_v
    traj_x(end+1) = x_est(1); 
    traj_y(end+1) = x_est(2); 
    traj_u(end+1) = 0.9 * cos(x_est(3));
    traj_v(end+1) = 0.9 * sin(x_est(3));
    % Enforce sliding window
    if length(traj_x) > max_trail_points
        traj_x(1) = [];
        traj_y(1) = [];
        traj_u(1) = [];
        traj_v(1) = [];
    end

    set(line_est, 'XData', traj_x, 'YData', traj_y);
    set(dot_est, 'XData', x_est(1), 'YData', x_est(2));

    % Keep quiver inputs strictly aligned in size to avoid runtime plotting
    % errors when the buffers are reshaped during updates.
    qx = traj_x(:);
    qy = traj_y(:);
    qu = traj_u(:);
    qv = traj_v(:);
    n_quiv = min([numel(qx), numel(qy), numel(qu), numel(qv)]);
    if n_quiv >= 1
        set(quiv_est, ...
            'XData', qx(1:n_quiv), ...
            'YData', qy(1:n_quiv), ...
            'UData', qu(1:n_quiv), ...
            'VData', qv(1:n_quiv));
    else
        set(quiv_est, 'XData', NaN, 'YData', NaN, 'UData', NaN, 'VData', NaN);
    end

    % Most-probable plotting failure point: scatter requires CData length
    % to match X/Y. Enforce consistent vector lengths before set().
    anchors_x = rr.anchors_xyz(:,1);
    anchors_y = rr.anchors_xyz(:,2);
    anchor_c = sigma2_used(:);
    n_anchor_plot = min([numel(anchors_x), numel(anchors_y), numel(anchor_c)]);

    if n_anchor_plot >= 1
        anchors_x = anchors_x(1:n_anchor_plot);
        anchors_y = anchors_y(1:n_anchor_plot);
        anchor_c = anchor_c(1:n_anchor_plot);

        finite_mask = isfinite(anchors_x) & isfinite(anchors_y) & isfinite(anchor_c);
        anchors_x = anchors_x(finite_mask);
        anchors_y = anchors_y(finite_mask);
        anchor_c = anchor_c(finite_mask);

        if isempty(anchors_x)
            set(h_used_anchors, 'XData', NaN, 'YData', NaN, 'CData', NaN);
        else
            set(h_used_anchors, 'XData', anchors_x, 'YData', anchors_y, 'CData', anchor_c);
            anchor_var_min_m2 = min(anchor_var_min_m2, min(anchor_c));
            anchor_var_max_m2 = max(anchor_var_max_m2, max(anchor_c));
        end
    else
        set(h_used_anchors, 'XData', NaN, 'YData', NaN, 'CData', NaN);
    end
    if isfinite(anchor_var_min_m2) && isfinite(anchor_var_max_m2)
        if anchor_var_max_m2 <= anchor_var_min_m2
            clim(ax_main, [anchor_var_min_m2, anchor_var_min_m2 + 1e-6]);
        else
            clim(ax_main, [anchor_var_min_m2, anchor_var_max_m2]);
        end
    end

    [ell_x, ell_y] = covariance_ellipse_points(x_est(1:2), P_est(1:2,1:2), ...
        viz_cfg.cov_ellipse_sigma_mult, viz_cfg.cov_ellipse_points);
    set(h_cov_ellipse, 'XData', ell_x, 'YData', ell_y);

    heading_sigma = sqrt(max(P_est(3,3), 0));
    [cone_x, cone_y] = heading_cone_points(x_est(1:2), x_est(3), heading_sigma, ...
        viz_cfg.heading_sigma_mult, viz_cfg.heading_cone_length_m, viz_cfg.heading_cone_points);
    set(h_heading_cone, 'XData', cone_x, 'YData', cone_y);

    cov_xy_trace = trace(P_est(1:2,1:2));
    traj_eval_error_m = NaN;
    [traj_eval, traj_eval_row] = append_traj_eval_log( ...
        traj_eval, x_est, P_est, rr, k_loop, mode_str, note_str, used_count, cov_xy_trace);
    if traj_eval.enabled
        traj_eval_error_m = traj_eval_row.distance_to_rect_m;
        update_traj_eval_plot_handles(traj_eval_row, x_est, h_traj_eval_closest, h_traj_eval_error);
    end

    if isnan(rr.round_id)
        r_print = -1;
    else
        r_print = rr.round_id;
    end
    mode_char = char(mode_str);

    if isfinite(traj_eval_error_m)
        title(sprintf('Real-Time UWB EKF | mode=%s | r=%d | pos=(%.2f, %.2f) | rect err=%.2fm', ...
            upper(mode_char), r_print, x_est(1), x_est(2), traj_eval_error_m));
        set(status_text, 'String', sprintf('k=%d | round=%d | mode=%s | note=%s | anchors=%d | rect err=%.2fm', ...
            k_loop, r_print, mode_char, char(note_str), used_count, traj_eval_error_m));
    else
        title(sprintf('Real-Time UWB EKF | mode=%s | r=%d | pos=(%.2f, %.2f)', upper(mode_char), r_print, x_est(1), x_est(2)));
        set(status_text, 'String', sprintf('k=%d | round=%d | mode=%s | note=%s | anchors=%d', ...
            k_loop, r_print, mode_char, char(note_str), used_count));
    end
    drawnow limitrate;
    gif_recorder = map_gif_recorder('capture', gif_recorder, map_fig);

    fprintf('[track] r=%d k=%d mode=%s note=%s raw/used=%d/%d pos=(%.2f,%.2f) v=%.2f w=%.2f cov=%.3f\n', ...
        r_print, k_loop, mode_char, char(note_str), rr.raw_count, used_count, ...
        x_est(1), x_est(2), x_est(4), x_est(5), cov_xy_trace);
end

%% Save TRAJ_EVAL logs + recorded map GIF
traj_eval = finalize_traj_eval_log(traj_eval);
map_gif_recorder('prompt_save', gif_recorder);

%% Local functions
function traj_eval = init_traj_eval(cfg, map, dT)
%INIT_TRAJ_EVAL Build the rectilinear reference and prepare per-run logs.
    traj_eval = empty_traj_eval_state();
    traj_eval.enabled = logical(get_cfg_value(cfg, 'enabled', true));
    if ~traj_eval.enabled
        return;
    end

    anchor_ids = double(get_cfg_value(cfg, 'rect_anchor_node_ids', [108, 123]));
    if numel(anchor_ids) ~= 2
        error('[traj_eval] rect_anchor_node_ids must contain exactly two node ids.');
    end

    [p0_xy, p0_label] = map_node_xy(map, anchor_ids(1));
    [p1_xy, p1_label] = map_node_xy(map, anchor_ids(2));
    ref_vec = p1_xy - p0_xy;
    path_length_m = norm(ref_vec);
    if path_length_m <= 1e-6
        error('[traj_eval] rectilinear reference endpoints are coincident.');
    end

    traj_eval.reference = struct();
    traj_eval.reference.anchor_node_ids = anchor_ids(:).';
    traj_eval.reference.anchor_labels = {p0_label, p1_label};
    traj_eval.reference.points_xy = [p0_xy; p1_xy];
    traj_eval.reference.unit_xy = ref_vec ./ path_length_m;
    traj_eval.reference.length_m = path_length_m;
    traj_eval.reference.heading_rad = atan2(ref_vec(2), ref_vec(1));
    traj_eval.dT = dT;
    traj_eval.created_at = datestr(now, 'yyyy-mm-dd HH:MM:SS');
    traj_eval.logging_enabled = logical(get_cfg_value(cfg, 'logging_enabled', true));

    if ~traj_eval.logging_enabled
        return;
    end

    logs_root_dir = char(get_cfg_value(cfg, 'logs_root_dir', fullfile(pwd, 'TRAJ_EVAL_logs')));
    if isempty(strtrim(logs_root_dir))
        logs_root_dir = fullfile(pwd, 'TRAJ_EVAL_logs');
    end
    if exist(logs_root_dir, 'dir') ~= 7
        mkdir(logs_root_dir);
    end

    session_prefix = char(get_cfg_value(cfg, 'session_prefix', 'traj_eval_rectilinear'));
    session_prefix = regexprep(strtrim(session_prefix), '[^\w.-]', '_');
    if isempty(session_prefix)
        session_prefix = 'traj_eval_rectilinear';
    end
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    session_name = sprintf('%s_nodes_%d_%d_%s', session_prefix, anchor_ids(1), anchor_ids(2), timestamp);
    session_dir = fullfile(logs_root_dir, session_name);
    if exist(session_dir, 'dir') ~= 7
        mkdir(session_dir);
    end

    traj_eval.logs_root_dir = logs_root_dir;
    traj_eval.session_dir = session_dir;
    traj_eval.csv_path = fullfile(session_dir, 'samples.csv');
    traj_eval.summary_path = fullfile(session_dir, 'summary.txt');
    traj_eval.summary_csv_path = fullfile(session_dir, 'summary.csv');
    traj_eval.mat_path = fullfile(session_dir, 'traj_eval_session.mat');
    write_traj_eval_csv_header(traj_eval.csv_path);
    write_traj_eval_metadata(traj_eval, 'init');
end

function traj_eval = empty_traj_eval_state()
    traj_eval = struct();
    traj_eval.enabled = false;
    traj_eval.logging_enabled = false;
    traj_eval.reference = struct();
    traj_eval.dT = NaN;
    traj_eval.created_at = '';
    traj_eval.start_round_id = NaN;
    traj_eval.row_count = 0;
    traj_eval.logs_root_dir = '';
    traj_eval.session_dir = '';
    traj_eval.csv_path = '';
    traj_eval.summary_path = '';
    traj_eval.summary_csv_path = '';
    traj_eval.mat_path = '';
end

function value = get_cfg_value(cfg, field_name, default_value)
    value = default_value;
    if isstruct(cfg) && isfield(cfg, field_name) && ~isempty(cfg.(field_name))
        value = cfg.(field_name);
    end
end

function [xy, label] = map_node_xy(map, node_id)
    idx = find(double(map.NodeId) == double(node_id), 1);
    if isempty(idx)
        error('[traj_eval] node %d is not present in the current DEPT tracking map.', node_id);
    end
    xy = [double(map.lat(idx)), double(map.lon(idx))];
    label = sprintf('node_%d', node_id);
end

function write_traj_eval_csv_header(csv_path)
    fid = fopen(csv_path, 'w');
    if fid < 0
        error('[traj_eval] could not create log file: %s', csv_path);
    end
    cleaner = onCleanup(@() fclose(fid));
    fprintf(fid, ['sample_idx,round_id,relative_round_idx,time_s,' ...
        'est_x_m,est_y_m,est_theta_rad,est_v_mps,est_omega_radps,' ...
        'closest_rect_x_m,closest_rect_y_m,distance_to_rect_m,' ...
        'signed_cross_track_m,abs_cross_track_m,' ...
        'along_track_m,unclamped_along_track_m,rect_fraction,rect_fraction_unclamped,projection_clamped,' ...
        'cov_xx_m2,cov_yy_m2,cov_xy_m2,cov_xy_trace_m2,' ...
        'raw_anchor_count,used_anchor_count,mode,note\n']);
end

function write_traj_eval_metadata(traj_eval, phase)
    if ~traj_eval.logging_enabled || isempty(traj_eval.session_dir)
        return;
    end

    metadata_path = fullfile(traj_eval.session_dir, 'metadata.txt');
    if strcmp(phase, 'init')
        fid = fopen(metadata_path, 'w');
    else
        fid = fopen(metadata_path, 'a');
    end
    if fid < 0
        fprintf('[traj_eval] warning: could not write metadata file %s\n', metadata_path);
        return;
    end
    cleaner = onCleanup(@() fclose(fid));

    ref = traj_eval.reference;
    if strcmp(phase, 'init')
        fprintf(fid, 'TRAJ_EVAL rectilinear session\n');
        fprintf(fid, 'created_at,%s\n', traj_eval.created_at);
        fprintf(fid, 'anchor_start_node_id,%d\n', ref.anchor_node_ids(1));
        fprintf(fid, 'anchor_end_node_id,%d\n', ref.anchor_node_ids(2));
        fprintf(fid, 'start_x_m,%.6f\n', ref.points_xy(1,1));
        fprintf(fid, 'start_y_m,%.6f\n', ref.points_xy(1,2));
        fprintf(fid, 'end_x_m,%.6f\n', ref.points_xy(2,1));
        fprintf(fid, 'end_y_m,%.6f\n', ref.points_xy(2,2));
        fprintf(fid, 'path_length_m,%.6f\n', ref.length_m);
        fprintf(fid, 'heading_rad,%.9f\n', ref.heading_rad);
        fprintf(fid, 'heading_deg,%.6f\n', rad2deg(ref.heading_rad));
        fprintf(fid, 'samples_csv,%s\n', traj_eval.csv_path);
    elseif strcmp(phase, 'start_round')
        fprintf(fid, 'start_round_id,%.0f\n', traj_eval.start_round_id);
    end
end

function traj_eval = set_traj_eval_start_round(traj_eval, round_id)
    if ~traj_eval.enabled
        return;
    end
    traj_eval.start_round_id = round_id;
    write_traj_eval_metadata(traj_eval, 'start_round');
end

function [traj_eval, row] = append_traj_eval_log( ...
    traj_eval, x_est, P_est, rr, k_loop, mode_str, note_str, used_count, cov_xy_trace)
%APPEND_TRAJ_EVAL_LOG Append one EKF sample and its nearest rectilinear point.
    row = empty_traj_eval_row();
    if ~traj_eval.enabled
        return;
    end

    row.sample_idx = traj_eval.row_count;
    row.round_id = struct_numeric_field(rr, 'round_id', NaN);
    if isfinite(row.round_id) && isfinite(traj_eval.start_round_id)
        row.relative_round_idx = row.round_id - traj_eval.start_round_id;
    else
        row.relative_round_idx = k_loop;
    end
    row.time_s = k_loop * traj_eval.dT;

    row.est_x_m = x_est(1);
    row.est_y_m = x_est(2);
    row.est_theta_rad = x_est(3);
    row.est_v_mps = x_est(4);
    row.est_omega_radps = x_est(5);

    geom = closest_rectilinear_point([x_est(1), x_est(2)], traj_eval.reference);
    row.closest_rect_x_m = geom.closest_xy(1);
    row.closest_rect_y_m = geom.closest_xy(2);
    row.distance_to_rect_m = geom.distance_to_segment_m;
    row.signed_cross_track_m = geom.signed_cross_track_m;
    row.abs_cross_track_m = abs(geom.signed_cross_track_m);
    row.along_track_m = geom.along_track_m;
    row.unclamped_along_track_m = geom.unclamped_along_track_m;
    row.rect_fraction = geom.fraction;
    row.rect_fraction_unclamped = geom.fraction_unclamped;
    row.projection_clamped = double(geom.projection_clamped);

    if ~isempty(P_est) && all(size(P_est) >= [2, 2])
        row.cov_xx_m2 = P_est(1,1);
        row.cov_yy_m2 = P_est(2,2);
        row.cov_xy_m2 = P_est(1,2);
    end
    row.cov_xy_trace_m2 = cov_xy_trace;
    row.raw_anchor_count = struct_numeric_field(rr, 'raw_count', NaN);
    row.used_anchor_count = used_count;
    row.mode = sanitize_csv_token(mode_str);
    row.note = sanitize_csv_token(note_str);

    if traj_eval.logging_enabled
        append_traj_eval_csv_row(traj_eval.csv_path, row);
    end
    traj_eval.row_count = traj_eval.row_count + 1;
end

function row = empty_traj_eval_row()
    row = struct();
    row.sample_idx = NaN;
    row.round_id = NaN;
    row.relative_round_idx = NaN;
    row.time_s = NaN;
    row.est_x_m = NaN;
    row.est_y_m = NaN;
    row.est_theta_rad = NaN;
    row.est_v_mps = NaN;
    row.est_omega_radps = NaN;
    row.closest_rect_x_m = NaN;
    row.closest_rect_y_m = NaN;
    row.distance_to_rect_m = NaN;
    row.signed_cross_track_m = NaN;
    row.abs_cross_track_m = NaN;
    row.along_track_m = NaN;
    row.unclamped_along_track_m = NaN;
    row.rect_fraction = NaN;
    row.rect_fraction_unclamped = NaN;
    row.projection_clamped = NaN;
    row.cov_xx_m2 = NaN;
    row.cov_yy_m2 = NaN;
    row.cov_xy_m2 = NaN;
    row.cov_xy_trace_m2 = NaN;
    row.raw_anchor_count = NaN;
    row.used_anchor_count = NaN;
    row.mode = '';
    row.note = '';
end

function value = struct_numeric_field(s, field_name, default_value)
    value = default_value;
    if isstruct(s) && isfield(s, field_name) && ~isempty(s.(field_name))
        value = double(s.(field_name));
    end
end

function token = sanitize_csv_token(value)
    token = char(value);
    token = strtrim(token);
    token = strrep(token, ',', ';');
    token = strrep(token, sprintf('\n'), ' ');
    token = strrep(token, sprintf('\r'), ' ');
end

function append_traj_eval_csv_row(csv_path, row)
    fid = fopen(csv_path, 'a');
    if fid < 0
        fprintf('[traj_eval] warning: could not append to log file %s\n', csv_path);
        return;
    end
    cleaner = onCleanup(@() fclose(fid));
    fprintf(fid, ['%.0f,%.0f,%.0f,%.6f,' ...
        '%.9f,%.9f,%.9f,%.9f,%.9f,' ...
        '%.9f,%.9f,%.9f,' ...
        '%.9f,%.9f,' ...
        '%.9f,%.9f,%.9f,%.9f,%.0f,' ...
        '%.9f,%.9f,%.9f,%.9f,' ...
        '%.0f,%.0f,%s,%s\n'], ...
        row.sample_idx, row.round_id, row.relative_round_idx, row.time_s, ...
        row.est_x_m, row.est_y_m, row.est_theta_rad, row.est_v_mps, row.est_omega_radps, ...
        row.closest_rect_x_m, row.closest_rect_y_m, row.distance_to_rect_m, ...
        row.signed_cross_track_m, row.abs_cross_track_m, ...
        row.along_track_m, row.unclamped_along_track_m, row.rect_fraction, row.rect_fraction_unclamped, row.projection_clamped, ...
        row.cov_xx_m2, row.cov_yy_m2, row.cov_xy_m2, row.cov_xy_trace_m2, ...
        row.raw_anchor_count, row.used_anchor_count, row.mode, row.note);
end

function geom = closest_rectilinear_point(point_xy, ref)
%CLOSEST_RECTILINEAR_POINT Project a point onto the finite reference segment.
    p = point_xy(:).';
    p0 = ref.points_xy(1,:);
    u = ref.unit_xy(:).';
    path_length_m = ref.length_m;

    rel = p - p0;
    unclamped_along_m = dot(rel, u);
    fraction_unclamped = unclamped_along_m / path_length_m;
    fraction = min(max(fraction_unclamped, 0.0), 1.0);
    along_m = fraction * path_length_m;
    closest_xy = p0 + along_m * u;

    geom = struct();
    geom.closest_xy = closest_xy;
    geom.distance_to_segment_m = norm(p - closest_xy);
    geom.signed_cross_track_m = u(1) * rel(2) - u(2) * rel(1);
    geom.unclamped_along_track_m = unclamped_along_m;
    geom.along_track_m = along_m;
    geom.fraction_unclamped = fraction_unclamped;
    geom.fraction = fraction;
    geom.projection_clamped = fraction ~= fraction_unclamped;
end

function update_traj_eval_plot_handles(row, x_est, h_closest, h_error)
    if isempty(row) || ~isstruct(row) || ~isfinite(row.closest_rect_x_m)
        return;
    end
    if ~isempty(h_closest) && isgraphics(h_closest)
        set(h_closest, 'XData', row.closest_rect_x_m, 'YData', row.closest_rect_y_m);
    end
    if ~isempty(h_error) && isgraphics(h_error)
        set(h_error, ...
            'XData', [x_est(1), row.closest_rect_x_m], ...
            'YData', [x_est(2), row.closest_rect_y_m]);
    end
end

function traj_eval = finalize_traj_eval_log(traj_eval)
%FINALIZE_TRAJ_EVAL_LOG Save statistics for post-run tracking evaluation.
    if ~traj_eval.enabled || ~traj_eval.logging_enabled || isempty(traj_eval.csv_path)
        return;
    end
    if exist(traj_eval.csv_path, 'file') ~= 2
        fprintf('[traj_eval] no CSV log found; summary skipped\n');
        return;
    end

    try
        samples = readtable(traj_eval.csv_path);
    catch ME
        fprintf('[traj_eval] could not read samples CSV for summary: %s\n', ME.message);
        return;
    end

    if isempty(samples) || height(samples) < 1
        fprintf('[traj_eval] no samples logged; summary skipped\n');
        return;
    end

    summary = compute_traj_eval_summary(samples, traj_eval.reference);
    write_traj_eval_summary_txt(traj_eval.summary_path, summary, traj_eval);
    write_traj_eval_summary_csv(traj_eval.summary_csv_path, summary);
    save(traj_eval.mat_path, 'traj_eval', 'summary', 'samples');

    fprintf('[traj_eval] saved %d samples in %s\n', height(samples), traj_eval.session_dir);
    fprintf('[traj_eval] rect distance std=%.3f m | rmse=%.3f m | p95=%.3f m\n', ...
        summary.distance_to_rect_m.std, summary.distance_to_rect_m.rmse, summary.distance_to_rect_m.p95);
end

function summary = compute_traj_eval_summary(samples, ref)
    valid = isfinite(samples.distance_to_rect_m);
    err = samples.distance_to_rect_m(valid);
    signed_cross = samples.signed_cross_track_m(valid);
    abs_cross = abs(signed_cross);
    along = samples.along_track_m(valid);

    summary = struct();
    summary.sample_count = numel(err);
    summary.path_length_m = ref.length_m;
    summary.reference_heading_rad = ref.heading_rad;
    summary.reference_heading_deg = rad2deg(ref.heading_rad);
    summary.distance_to_rect_m = vector_stats(err);
    summary.signed_cross_track_m = vector_stats(signed_cross);
    summary.abs_cross_track_m = vector_stats(abs_cross);
    summary.along_track_m = vector_stats(along);
    summary.est_x_m = vector_stats(samples.est_x_m(valid));
    summary.est_y_m = vector_stats(samples.est_y_m(valid));
    summary.round = round_stats(samples);
    summary.linear_fit = total_least_squares_line_summary(samples, valid, ref);
end

function s = vector_stats(x)
    x = x(:);
    x = x(isfinite(x));
    s = struct('n', numel(x), 'mean', NaN, 'std', NaN, 'var', NaN, ...
        'rmse', NaN, 'median', NaN, 'p95', NaN, 'min', NaN, 'max', NaN);
    if isempty(x)
        return;
    end
    s.mean = mean(x);
    s.std = std(x);
    s.var = var(x);
    s.rmse = sqrt(mean(x.^2));
    s.median = median(x);
    s.p95 = percentile_no_toolbox(x, 95);
    s.min = min(x);
    s.max = max(x);
end

function p = percentile_no_toolbox(x, pct)
    x = sort(x(:));
    n = numel(x);
    if n == 0
        p = NaN;
        return;
    end
    if n == 1
        p = x(1);
        return;
    end
    q = min(max(pct / 100.0, 0.0), 1.0);
    pos = 1 + (n - 1) * q;
    lo = floor(pos);
    hi = ceil(pos);
    if lo == hi
        p = x(lo);
    else
        p = x(lo) + (pos - lo) * (x(hi) - x(lo));
    end
end

function s = round_stats(samples)
    finite_round = samples.round_id(isfinite(samples.round_id));
    finite_rel = samples.relative_round_idx(isfinite(samples.relative_round_idx));
    s = struct('start_round_id', NaN, 'end_round_id', NaN, 'first_relative_round_idx', NaN, ...
        'last_relative_round_idx', NaN, 'sample_count', height(samples));
    if ~isempty(finite_round)
        s.start_round_id = min(finite_round);
        s.end_round_id = max(finite_round);
    end
    if ~isempty(finite_rel)
        s.first_relative_round_idx = min(finite_rel);
        s.last_relative_round_idx = max(finite_rel);
    end
end

function fit = total_least_squares_line_summary(samples, valid, ref)
    fit = struct('heading_rad', NaN, 'heading_deg', NaN, 'heading_error_rad', NaN, ...
        'heading_error_deg', NaN, 'mean_reference_signed_offset_m', NaN, ...
        'residual_to_fit_m', vector_stats([]));

    xy = [samples.est_x_m(valid), samples.est_y_m(valid)];
    xy = xy(all(isfinite(xy), 2), :);
    if size(xy, 1) < 2
        return;
    end

    mu = mean(xy, 1);
    centered = xy - mu;
    C = centered.' * centered / max(size(centered, 1) - 1, 1);
    [V, D] = eig(0.5 * (C + C.'));
    [~, idx] = max(diag(D));
    direction = V(:, idx);
    if norm(direction) <= 1e-12 || any(~isfinite(direction))
        return;
    end
    theta_fit = atan2(direction(2), direction(1));
    theta_fit = choose_axis_heading_direction(theta_fit, ref.heading_rad);
    u_fit = [cos(theta_fit), sin(theta_fit)];

    residual = u_fit(1) * (xy(:,2) - mu(2)) - u_fit(2) * (xy(:,1) - mu(1));
    ref_u = ref.unit_xy(:).';
    ref_p0 = ref.points_xy(1,:);
    ref_offset = ref_u(1) * (xy(:,2) - ref_p0(2)) - ref_u(2) * (xy(:,1) - ref_p0(1));

    fit.heading_rad = theta_fit;
    fit.heading_deg = rad2deg(theta_fit);
    fit.heading_error_rad = wrap_angle_pi(theta_fit - ref.heading_rad);
    fit.heading_error_deg = rad2deg(fit.heading_error_rad);
    fit.mean_reference_signed_offset_m = mean(ref_offset);
    fit.residual_to_fit_m = vector_stats(residual);
end

function write_traj_eval_summary_txt(summary_path, summary, traj_eval)
    fid = fopen(summary_path, 'w');
    if fid < 0
        fprintf('[traj_eval] warning: could not write summary file %s\n', summary_path);
        return;
    end
    cleaner = onCleanup(@() fclose(fid));

    ref = traj_eval.reference;
    fprintf(fid, 'TRAJ_EVAL rectilinear tracking summary\n');
    fprintf(fid, 'session_dir: %s\n', traj_eval.session_dir);
    fprintf(fid, 'samples_csv: %s\n', traj_eval.csv_path);
    fprintf(fid, 'reference_nodes: %d -> %d\n', ref.anchor_node_ids(1), ref.anchor_node_ids(2));
    fprintf(fid, 'reference_start_xy_m: %.6f, %.6f\n', ref.points_xy(1,1), ref.points_xy(1,2));
    fprintf(fid, 'reference_end_xy_m: %.6f, %.6f\n', ref.points_xy(2,1), ref.points_xy(2,2));
    fprintf(fid, 'reference_length_m: %.6f\n', summary.path_length_m);
    fprintf(fid, 'reference_heading_deg: %.6f\n\n', summary.reference_heading_deg);

    print_stats(fid, 'distance_to_rect_m', summary.distance_to_rect_m);
    print_stats(fid, 'signed_cross_track_m', summary.signed_cross_track_m);
    print_stats(fid, 'abs_cross_track_m', summary.abs_cross_track_m);
    print_stats(fid, 'along_track_m', summary.along_track_m);
    print_stats(fid, 'est_x_m', summary.est_x_m);
    print_stats(fid, 'est_y_m', summary.est_y_m);

    fprintf(fid, '\nround_start: %.0f\n', summary.round.start_round_id);
    fprintf(fid, 'round_end: %.0f\n', summary.round.end_round_id);
    fprintf(fid, 'relative_round_first: %.0f\n', summary.round.first_relative_round_idx);
    fprintf(fid, 'relative_round_last: %.0f\n', summary.round.last_relative_round_idx);

    fprintf(fid, '\nlinear_fit_heading_deg: %.6f\n', summary.linear_fit.heading_deg);
    fprintf(fid, 'linear_fit_heading_error_deg: %.6f\n', summary.linear_fit.heading_error_deg);
    fprintf(fid, 'linear_fit_mean_reference_signed_offset_m: %.6f\n', ...
        summary.linear_fit.mean_reference_signed_offset_m);
    print_stats(fid, 'linear_fit_residual_to_fit_m', summary.linear_fit.residual_to_fit_m);
end

function print_stats(fid, label, s)
    fprintf(fid, '\n%s\n', label);
    fprintf(fid, '  n: %.0f\n', s.n);
    fprintf(fid, '  mean: %.6f\n', s.mean);
    fprintf(fid, '  std: %.6f\n', s.std);
    fprintf(fid, '  var: %.6f\n', s.var);
    fprintf(fid, '  rmse: %.6f\n', s.rmse);
    fprintf(fid, '  median: %.6f\n', s.median);
    fprintf(fid, '  p95: %.6f\n', s.p95);
    fprintf(fid, '  min: %.6f\n', s.min);
    fprintf(fid, '  max: %.6f\n', s.max);
end

function write_traj_eval_summary_csv(summary_csv_path, summary)
    fid = fopen(summary_csv_path, 'w');
    if fid < 0
        fprintf('[traj_eval] warning: could not write summary CSV %s\n', summary_csv_path);
        return;
    end
    cleaner = onCleanup(@() fclose(fid));
    fprintf(fid, ['sample_count,path_length_m,reference_heading_deg,' ...
        'distance_mean_m,distance_std_m,distance_var_m2,distance_rmse_m,distance_median_m,distance_p95_m,distance_max_m,' ...
        'signed_cross_mean_m,signed_cross_std_m,signed_cross_var_m2,' ...
        'abs_cross_mean_m,abs_cross_std_m,abs_cross_p95_m,' ...
        'fit_heading_deg,fit_heading_error_deg,fit_mean_reference_signed_offset_m,fit_residual_std_m\n']);
    fprintf(fid, ['%.0f,%.9f,%.9f,' ...
        '%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,' ...
        '%.9f,%.9f,%.9f,' ...
        '%.9f,%.9f,%.9f,' ...
        '%.9f,%.9f,%.9f,%.9f\n'], ...
        summary.sample_count, summary.path_length_m, summary.reference_heading_deg, ...
        summary.distance_to_rect_m.mean, summary.distance_to_rect_m.std, summary.distance_to_rect_m.var, ...
        summary.distance_to_rect_m.rmse, summary.distance_to_rect_m.median, summary.distance_to_rect_m.p95, ...
        summary.distance_to_rect_m.max, ...
        summary.signed_cross_track_m.mean, summary.signed_cross_track_m.std, summary.signed_cross_track_m.var, ...
        summary.abs_cross_track_m.mean, summary.abs_cross_track_m.std, summary.abs_cross_track_m.p95, ...
        summary.linear_fit.heading_deg, summary.linear_fit.heading_error_deg, ...
        summary.linear_fit.mean_reference_signed_offset_m, summary.linear_fit.residual_to_fit_m.std);
end

function H = build_range_jacobian_unicycle(xk, anchors_xyz, z_fixed_m)
%BUILD_RANGE_JACOBIAN_UNICYCLE Jacobian of range model wrt [px py theta v omega].
    n = size(anchors_xyz, 1);
    H = zeros(n, 5);
    dx = xk(1) - anchors_xyz(:,1);
    dy = xk(2) - anchors_xyz(:,2);
    dz = z_fixed_m - anchors_xyz(:,3);
    rr = sqrt(dx.^2 + dy.^2 + dz.^2);
    rr = max(rr, 1e-6);
    H(:,1) = dx ./ rr;   % d(range)/d(px)
    H(:,2) = dy ./ rr;   % d(range)/d(py)
% columns 3,4,5 = 0 — no direct observability of theta, v, omega
end

function [x0, P0, info] = initialize_state_weighted( ...
    s, map, addr_short, noise_map, bias_map, default_noise, z_fixed_m, cfg_init, ...
    room_constraint, use_room_constraint, room_projection_threshold_m)
%INITIALIZE_STATE_WEIGHTED
% Wrapper that supports "single" and "multi" round initialization modes.
    mode_str = 'single';
    if isfield(cfg_init, 'mode') && strlength(string(cfg_init.mode)) > 0
        mode_str = lower(char(string(cfg_init.mode)));
    end

    if strcmp(mode_str, 'multi')
        [x0, P0, info] = initialize_state_weighted_multi_round( ...
            s, map, addr_short, noise_map, bias_map, default_noise, z_fixed_m, cfg_init, ...
            room_constraint, use_room_constraint, room_projection_threshold_m);
        return;
    end

    [x0, P0, info] = initialize_state_weighted_single_round( ...
        s, map, addr_short, noise_map, bias_map, default_noise, z_fixed_m, cfg_init, ...
        room_constraint, use_room_constraint, room_projection_threshold_m);
end

function [x0, P0, info] = initialize_state_weighted_single_round( ...
    s, map, addr_short, noise_map, bias_map, default_noise, z_fixed_m, cfg_init, ...
    room_constraint, use_room_constraint, room_projection_threshold_m)
%INITIALIZE_STATE_WEIGHTED_SINGLE_ROUND
% Collect the first valid round and compute x0/P0 from one weighted trilateration using all valid anchor measurements.
    ok = false;
    rr = struct('round_id', NaN, 'raw_count', 0, 'used_count', 0, ...
        'anchors_xyz', zeros(0,3), 'distances_m', zeros(0,1), 'std_m', zeros(0,1));
    xy = [NaN; NaN];
    Pxy = [];
    mae = NaN;

    last_attempt = 0;
    for attempt = 1:cfg_init.max_init_attempts
        last_attempt = attempt;
        rr = collect_round_measurements_all( ...
            s, map, addr_short, noise_map, bias_map, default_noise);

        [xy, Pxy, mae, ok] = weighted_trilateration_wls_all( ...
            rr.anchors_xyz, rr.distances_m, rr.std_m, z_fixed_m, [], cfg_init.range_variance_distance_gain);

        fprintf('[init] try %d/%d round=%d raw/used=%d/%d ok=%d mae=%.3f\n', ...
            attempt, cfg_init.max_init_attempts, rr.round_id, rr.raw_count, rr.used_count, ok, mae);
        if ok
            break;
        end
    end

    if ~ok
        error('[init] single-round initialization failed: no valid weighted trilateration.');
    end

    if use_room_constraint && ~isempty(room_constraint)
        [xy_room, info_room] = apply_room_constraint(room_constraint, xy.', ...
            'projection_threshold_m', room_projection_threshold_m, 'fallback_pose', xy.', 'label', 'main_new_init_single');
        if ~strcmp(info_room.action, 'reject')
            xy = xy_room(:);
            if strcmp(info_room.action, 'project')
                fprintf('[room] init projected to free map (dist=%.2f m)\n', info_room.distance_to_free_m);
            end
        else
            fprintf('[room] init rejected by map; keeping raw init\n');
        end
    end

    % With one round, displacement-based heading is not available.
    [theta0, sigma_theta, heading_source] = estimate_initial_heading_from_bootstrap( ...
        xy(:).', xy(:).', cfg_init);

    if isempty(Pxy) || any(~isfinite(Pxy(:)))
        Pxy = cfg_init.min_xy_variance_m2 * eye(2);
    end
    Pxy0 = 0.5 * (Pxy + Pxy.') + cfg_init.min_xy_variance_m2 * eye(2);
    Pxy0 = 0.5 * (Pxy0 + Pxy0.');

    x0 = [xy(1); xy(2); theta0; 0; 0];
    P0 = blkdiag(Pxy0, sigma_theta^2, cfg_init.initial_speed_std_mps^2, cfg_init.initial_yaw_rate_std_radps^2);

    info = struct();
    info.round_id = rr.round_id;
    info.raw_count = rr.raw_count;
    info.used_count = rr.used_count;
    info.mae_m = mae;
    info.mode = "single";
    info.accepted_rounds = 1;
    info.total_attempts = last_attempt;
    info.heading_source = heading_source;
end

function [x0, P0, info] = initialize_state_weighted_multi_round( ...
    s, map, addr_short, noise_map, bias_map, default_noise, z_fixed_m, cfg_init, ...
    room_constraint, use_room_constraint, room_projection_threshold_m)
%INITIALIZE_STATE_WEIGHTED_MULTI_ROUND
% Collect multiple rounds, solve WLS per round, reject outliers robustly,
% and aggregate into a more stable x0/P0 estimate.
    min_anchor_count = max(1, cfg_init.bootstrap_min_anchor_count);
    min_valid_rounds = max(1, cfg_init.bootstrap_min_valid_rounds);
    target_rounds = max(min_valid_rounds, cfg_init.bootstrap_target_rounds);

    xy_all = zeros(0, 2);
    mae_all = zeros(0, 1);
    Pxy_all = zeros(2, 2, 0);
    rr_last = struct('round_id', NaN, 'raw_count', 0, 'used_count', 0);
    attempts_done = 0;

    for attempt = 1:cfg_init.max_init_attempts
        attempts_done = attempt;
        rr = collect_round_measurements_all( ...
            s, map, addr_short, noise_map, bias_map, default_noise);
        rr_last = rr;

        if rr.used_count < min_anchor_count
            fprintf('[init] try %d/%d round=%d skipped used=%d (<%d)\n', ...
                attempt, cfg_init.max_init_attempts, rr.round_id, rr.used_count, min_anchor_count);
            continue;
        end

        [xy_i, Pxy_i, mae_i, ok_i] = weighted_trilateration_wls_all( ...
            rr.anchors_xyz, rr.distances_m, rr.std_m, z_fixed_m, [], cfg_init.range_variance_distance_gain);

        fprintf('[init] try %d/%d round=%d raw/used=%d/%d ok=%d mae=%.3f\n', ...
            attempt, cfg_init.max_init_attempts, rr.round_id, rr.raw_count, rr.used_count, ok_i, mae_i);

        if ~ok_i
            continue;
        end

        xy_all(end+1, :) = xy_i(:).';
        mae_all(end+1, 1) = mae_i;
        Pxy_all(:, :, end+1) = Pxy_i;

        if size(xy_all, 1) >= target_rounds
            break;
        end
    end

    n_valid = size(xy_all, 1);
    if n_valid < min_valid_rounds
        fprintf('[init] multi-round collected only %d valid rounds (min=%d), fallback to single-round\n', ...
            n_valid, min_valid_rounds);
        [x0, P0, info] = initialize_state_weighted_single_round( ...
            s, map, addr_short, noise_map, bias_map, default_noise, z_fixed_m, cfg_init, ...
            room_constraint, use_room_constraint, room_projection_threshold_m);
        info.mode = "single_fallback";
        info.accepted_rounds = n_valid;
        info.total_attempts = attempts_done;
        return;
    end

    inliers = bootstrap_inlier_mask(xy_all, mae_all, cfg_init);
    if nnz(inliers) < min_valid_rounds
        inliers = true(n_valid, 1);
    end

    idx_in = find(inliers);
    xy_in = xy_all(idx_in, :);
    mae_in = mae_all(idx_in);
    Pxy_in = Pxy_all(:, :, idx_in);

    cov_trace = squeeze(Pxy_in(1,1,:) + Pxy_in(2,2,:));
    cov_trace = reshape(cov_trace, [], 1);
    weights = 1 ./ max(cov_trace, cfg_init.min_xy_variance_m2);
    weights = weights ./ max(mae_in, 0.05);
    weights = weights ./ max(sum(weights), 1e-9);

    xy = [sum(weights .* xy_in(:,1)); sum(weights .* xy_in(:,2))];
    if use_room_constraint && ~isempty(room_constraint)
        [xy_room, info_room] = apply_room_constraint(room_constraint, xy.', ...
            'projection_threshold_m', room_projection_threshold_m, 'fallback_pose', xy.', 'label', 'main_new_init_multi');
        if ~strcmp(info_room.action, 'reject')
            xy = xy_room(:);
            if strcmp(info_room.action, 'project')
                fprintf('[room] init projected to free map (dist=%.2f m)\n', info_room.distance_to_free_m);
            end
        else
            fprintf('[room] init rejected by map; keeping raw init\n');
        end
    end

    % Weighted sample covariance + mean of per-round WLS covariance.
    diffs = xy_in - xy(:).';
    centered = diffs .* sqrt(weights);
    cov_boot = centered.' * centered;
    cov_wls = zeros(2,2);
    for i = 1:numel(idx_in)
        cov_wls = cov_wls + weights(i) * Pxy_in(:,:,i);
    end
    Pxy = cov_boot + cov_wls;
    Pxy0 = 0.5 * (Pxy + Pxy.') + cfg_init.min_xy_variance_m2 * eye(2);
    Pxy0 = 0.5 * (Pxy0 + Pxy0.');

    [theta0, sigma_theta, heading_source] = estimate_initial_heading_from_bootstrap(xy_in, xy(:).', cfg_init);

    x0 = [xy(1); xy(2); theta0; 0; 0];
    P0 = blkdiag(Pxy0, sigma_theta^2, cfg_init.initial_speed_std_mps^2, cfg_init.initial_yaw_rate_std_radps^2);

    mae_weighted = sum(weights .* mae_in);
    info = struct();
    info.round_id = rr_last.round_id;
    info.raw_count = rr_last.raw_count;
    info.used_count = rr_last.used_count;
    info.mae_m = mae_weighted;
    info.mode = "multi";
    info.accepted_rounds = nnz(inliers);
    info.total_attempts = attempts_done;
    info.heading_source = heading_source;
end

function inliers = bootstrap_inlier_mask(xy_all, mae_all, cfg_init)
%BOOTSTRAP_INLIER_MASK Robust inlier selection for multi-round init.
    n = size(xy_all, 1);
    if n == 0
        inliers = false(0,1);
        return;
    end

    mae_med = median(mae_all, 'omitnan');
    mae_mad = median(abs(mae_all - mae_med), 'omitnan');
    mae_gate = max(cfg_init.bootstrap_mae_gate_min_m, mae_med + cfg_init.bootstrap_mae_gate_k * max(mae_mad, 1e-6));
    in_mae = abs(mae_all - mae_med) <= mae_gate;

    xy_med = median(xy_all, 1, 'omitnan');
    dx = abs(xy_all(:,1) - xy_med(1));
    dy = abs(xy_all(:,2) - xy_med(2));
    mad_x = median(dx, 'omitnan');
    mad_y = median(dy, 'omitnan');
    gate_x = max(cfg_init.bootstrap_xy_gate_min_m, cfg_init.bootstrap_xy_gate_k * max(mad_x, 1e-6));
    gate_y = max(cfg_init.bootstrap_xy_gate_min_m, cfg_init.bootstrap_xy_gate_k * max(mad_y, 1e-6));
    in_xy = (dx <= gate_x) & (dy <= gate_y);

    inliers = in_mae & in_xy;
    if nnz(inliers) < 2
        inliers = true(n,1);
    end
end

function [theta0, sigma_theta, source] = estimate_initial_heading_from_bootstrap(xy_track, xy_ref, cfg_init)
%ESTIMATE_INITIAL_HEADING_FROM_BOOTSTRAP
% Prefer displacement-based heading from bootstrap samples, then optional
% corridor-axis fallback, then zero-heading fallback.
    theta0 = 0;
    sigma_theta = cfg_init.initial_heading_std_rad;
    source = "fallback_zero";

    if size(xy_track, 1) >= 2
        dxy = xy_track(end, :) - xy_track(1, :);
        span = norm(dxy);
        if span >= cfg_init.bootstrap_min_heading_span_m
            theta0 = atan2(dxy(2), dxy(1));
            sigma_theta = max(cfg_init.bootstrap_heading_std_floor_rad, ...
                cfg_init.initial_heading_std_rad / sqrt(size(xy_track, 1)));
            source = "bootstrap_displacement";
            return;
        end
    end

    if isfield(cfg_init, 'enable_corridor_heading_fallback') && cfg_init.enable_corridor_heading_fallback && ...
       isfield(cfg_init, 'corridor_segments_xyxy') && ~isempty(cfg_init.corridor_segments_xyxy)
        in_atrium = false;
        if isfield(cfg_init, 'disable_corridor_prior_in_atrium') && cfg_init.disable_corridor_prior_in_atrium && ...
           isfield(cfg_init, 'atrium_polygon_xy') && ~isempty(cfg_init.atrium_polygon_xy)
            poly = cfg_init.atrium_polygon_xy;
            in_atrium = inpolygon(xy_ref(1), xy_ref(2), poly(:,1), poly(:,2));
        end
        if ~in_atrium
            [in_corridor, theta_axis, ~, ~] = infer_corridor_axis_heading( ...
                xy_ref(:).', 0.0, cfg_init.corridor_segments_xyxy, cfg_init.corridor_half_width_m);
            if in_corridor
                theta0 = theta_axis;
                sigma_theta = min(cfg_init.initial_heading_std_rad, deg2rad(20));
                source = "corridor_axis_fallback";
                return;
            end
        end
    end
end

function rr = collect_round_measurements_all( ...
    s, map, addr_short, noise_map, bias_map, default_noise, stop_handle)
%COLLECT_ROUND_MEASUREMENTS_ALL
% Keep all measurements from all available anchors for the current round.
    if nargin < 7
        stop_handle = [];
    end

    while true
        if ~isempty(stop_handle) && ~isgraphics(stop_handle)
            rr = [];
            return;
        end

        [round_meas, round_id] = read_single_round(s, map, addr_short, noise_map, bias_map, default_noise);
        if isempty(round_meas)
            pause(0.01);
            continue;
        end

        idx = 1:numel(round_meas);
        rr = struct();
        rr.round_id = round_id;
        rr.raw_count = numel(round_meas);
        rr.used_count = numel(idx);
        rr.anchors_xyz = [[round_meas(idx).x].', [round_meas(idx).y].', [round_meas(idx).z].'];
        rr.distances_m = [round_meas(idx).dist_m].';
        rr.std_m = [round_meas(idx).noise].';
        return;
    end
end

function [round_meas, round_id] = read_single_round(s, map, addr_short, noise_map, bias_map, default_noise)
%READ_SINGLE_ROUND Parse one complete round from serial stream.
    round_meas = empty_round_meas();
    round_id = NaN;
    
    % Render this function more robust by checking if we're reading from an
    % empty raw line
    try
        % Use a very short check to see if bytes are actually available
        if s.NumBytesAvailable == 0
            return; 
        end
        raw_line = readline(s);
    catch
        fprintf('[serial] Warning: low-level transport error (SceneNode).\n');
        return;
    end

    if isempty(raw_line) || strlength(raw_line) == 0
        return;
    end

    while true
        line = strtrim(readline(s));
        [is_meas, meas] = parse_ranging_meas(line);

        if is_meas
            if isnan(round_id)
                round_id = meas.round_id;
            elseif meas.round_id ~= round_id
                round_id = meas.round_id;
                round_meas = empty_round_meas();
            end

            idx_map = find(strcmpi(addr_short, meas.addr), 1);
            if isempty(idx_map)
                continue;
            end

            node_id = double(map.NodeId(idx_map));
            if isKey(noise_map, node_id)
                meas_noise = noise_map(node_id);
            else
                meas_noise = default_noise;
            end

            if isKey(bias_map, node_id)
                meas_bias = bias_map(node_id);
            else
                meas_bias = 0.0;
            end

            entry = struct( ...
                'addr', meas.addr, ...
                'dist_m', meas.dist_m - meas_bias, ...
                'noise', meas_noise, ...
                'x', map.lat(idx_map), ...
                'y', map.lon(idx_map), ...
                'z', map.z(idx_map));

            idx_old = find(strcmpi({round_meas.addr}, meas.addr), 1);
            if isempty(idx_old)
                round_meas(end+1) = entry; 
            elseif should_replace_measurement(entry, round_meas(idx_old))
                round_meas(idx_old) = entry;
            end
            continue;
        end

        summary = parse_round_summary(line);
        if ~isnan(summary.round_id)
            if isnan(round_id)
                round_id = summary.round_id;
            end
            if summary.round_id >= round_id
                return;
            end
        end
    end
end

function meas = empty_round_meas()
    meas = struct('addr', {}, 'dist_m', {}, 'noise', {}, 'x', {}, 'y', {}, 'z', {});
end

function tf = should_replace_measurement(new_meas, old_meas)
%SHOULD_REPLACE_MEASUREMENT Keep the newest sample for duplicated anchor. 
    tf = true;
end

function [is_meas, meas] = parse_ranging_meas(line)
%PARSE_RANGING_MEAS Parse one "RANGING MEAS" line.
    expr = ['RANGING MEAS \[(?<round>\d+)\]\s+\[[\w:]+->(?<addr>[\w:]+)\]\s+' ...
        '(?<dist>\d+)\s+mm'];
    tok = regexp(char(line), expr, 'names');
    if isempty(tok)
        is_meas = false;
        meas = struct('round_id', NaN, 'addr', '', 'dist_m', NaN);
        return;
    end

    is_meas = true;
    meas = struct( ...
        'round_id', str2double(tok.round), ...
        'addr', lower(tok.addr), ...
        'dist_m', str2double(tok.dist) / 1000.0);
end

function summary = parse_round_summary(line)
%PARSE_ROUND_SUMMARY Parse round boundary line.
    summary = struct('round_id', NaN);
    tok = regexp(char(line), ...
        '^\[(?<round>\d+)\]\s+round:\s+(?<meas>\d+)\s+meas', 'names');
    if ~isempty(tok)
        summary.round_id = str2double(tok.round);
        return;
    end
    tok2 = regexp(char(line), 'ROUND SUMMARY \[(?<round>\d+)\]', 'names');
    if ~isempty(tok2)
        summary.round_id = str2double(tok2.round);
    end
end

function [xy, Pxy, mae, ok] = weighted_trilateration_wls_all(anchors_xyz, distances_m, std_m, z_fixed_m, x0, distance_variance_gain)
%WEIGHTED_TRILATERATION_WLS_ALL
% Weighted Gauss-Newton on all available valid measurements:
% min_x sum_i ( (h_i(x)-d_i)^2 / sigma_i^2 )
    xy = [NaN; NaN];
    Pxy = [];
    mae = NaN;
    ok = false;

    n = size(anchors_xyz, 1);
    if n < 1
        return;
    end

    sigma2 = range_variance_model(std_m(:), distances_m(:), distance_variance_gain);
    W = diag(1 ./ sigma2);

    if isempty(x0) || any(~isfinite(x0))
        weights = 1 ./ sigma2;
        weights = weights / sum(weights);
        xk = [sum(weights .* anchors_xyz(:,1)); sum(weights .* anchors_xyz(:,2))];
    else
        xk = x0(:);
    end

    lambda = 1e-4;
    for it = 1:15
        dx = xk(1) - anchors_xyz(:,1);
        dy = xk(2) - anchors_xyz(:,2);
        dz = z_fixed_m - anchors_xyz(:,3);
        hk = sqrt(dx.^2 + dy.^2 + dz.^2);
        hk = max(hk, 1e-6);

        res = distances_m(:) - hk;
        H = [dx ./ hk, dy ./ hk];

        N = H.' * W * H + lambda * eye(2);
        g = H.' * W * res;

        if rcond(N) < 1e-10
            step = pinv(N) * g;
        else
            step = N \ g;
        end

        xk = xk + step;
        if norm(step) < 1e-4
            break;
        end
    end

    dx = xk(1) - anchors_xyz(:,1);
    dy = xk(2) - anchors_xyz(:,2);
    dz = z_fixed_m - anchors_xyz(:,3);
    hk = sqrt(dx.^2 + dy.^2 + dz.^2);
    hk = max(hk, 1e-6);
    res = distances_m(:) - hk;
    H = [dx ./ hk, dy ./ hk];

    N = H.' * W * H + lambda * eye(2);
    if rcond(N) < 1e-10
        Pxy = pinv(N);
    else
        Pxy = inv(N);
    end

    xy = xk;
    mae = mean(abs(res), 'omitnan');
    ok = all(isfinite([xy; mae])) && all(isfinite(Pxy(:)));
end

function sigma2 = range_variance_model(sigma0_m, distance_m, distance_variance_gain)
%RANGE_VARIANCE_MODEL Per-anchor distance-dependent variance model:
% sigma_i^2(d) = sigma0_i^2 + k * d_i^2
    sigma0_sq = max(sigma0_m(:), 1e-3).^2;
    distance_sq = max(distance_m(:), 0).^2;
    sigma2 = sigma0_sq + distance_variance_gain .* distance_sq;
    sigma2 = max(sigma2, 1e-6);
end

function [x_ell, y_ell] = covariance_ellipse_points(mu_xy, P_xy, n_sigma, n_pts)
%COVARIANCE_ELLIPSE_POINTS Build a 2D covariance ellipse for plotting.
    x_ell = NaN;
    y_ell = NaN;

    P_sym = 0.5 * (P_xy + P_xy.');
    if any(~isfinite(P_sym(:)))
        return;
    end

    [V, D] = eig(P_sym);
    eigvals = diag(D);
    eigvals = max(eigvals, 0);
    radii = n_sigma * sqrt(eigvals);

    t = linspace(0, 2*pi, max(16, n_pts));
    circle_pts = [cos(t); sin(t)];
    ellipse_pts = V * diag(radii) * circle_pts + mu_xy(:);
    x_ell = ellipse_pts(1, :);
    y_ell = ellipse_pts(2, :);
end

function [x_cone, y_cone] = heading_cone_points(mu_xy, theta, sigma_theta, n_sigma, cone_len_m, n_arc_pts)
%HEADING_CONE_POINTS Build a heading-uncertainty cone polygon.
    half_angle = n_sigma * max(sigma_theta, 0);
    half_angle = min(half_angle, pi - 1e-3);

    arc_angles = linspace(theta - half_angle, theta + half_angle, max(6, n_arc_pts));
    arc_x = mu_xy(1) + cone_len_m * cos(arc_angles);
    arc_y = mu_xy(2) + cone_len_m * sin(arc_angles);

    x_cone = [mu_xy(1), arc_x, mu_xy(1)];
    y_cone = [mu_xy(2), arc_y, mu_xy(2)];
end

function [x_out, P_out, info] = apply_heading_prior_unicycle(x_pred, P_pred, x_prev_est, I5, cfg)
%APPLY_HEADING_PRIOR_UNICYCLE
% Heading pseudo-measurement for unicycle EKF.
%
% State is x=[x y theta v omega].
% We do a "virtual measurement" on theta:
%   z_theta ~= theta_target
% where theta_target comes from geometry (motion and/or corridor axis).
%
% Output:
% - x_out, P_out : predicted state/cov after heading pseudo-update
% - info         : debug info on what rule was used
    x_out = x_pred;
    P_out = P_pred;
    info = struct('applied', false, 'used_corridor_axis', false, ...
        'dist_moved_m', 0.0, 'corridor_id', 0, 'axis_distance_m', Inf, ...
        'in_atrium', false);

    if ~cfg.enabled
        return;
    end

    % Step 1: estimate heading from predicted displacement.
    dp = x_pred(1:2) - x_prev_est(1:2);
    dist_moved = norm(dp);
    info.dist_moved_m = dist_moved;
    % If movement is too small, displacement heading is noisy -> skip prior.
    if dist_moved < cfg.min_motion_for_heading_m
        return;
    end

    theta_motion = atan2(dp(2), dp(1));
    % Default target: motion heading (used outside corridors).
    theta_target = theta_motion;
    % Default confidence: "base" uncertainty.
    sigma_theta_used = cfg.base_heading_std_rad;

    % Step 2: optionally replace/blend with corridor axis heading.
    % If we are in the atrium polygon, corridor forcing is disabled.
    in_atrium = false;
    if isfield(cfg, 'disable_corridor_prior_in_atrium') && cfg.disable_corridor_prior_in_atrium && ...
       isfield(cfg, 'atrium_polygon_xy') && ~isempty(cfg.atrium_polygon_xy)
        poly = cfg.atrium_polygon_xy;
        in_atrium = inpolygon(x_pred(1), x_pred(2), poly(:,1), poly(:,2));
    end
    info.in_atrium = in_atrium;

    if in_atrium
        in_corridor = false;
        theta_axis = x_pred(3); 
        axis_dist = Inf;
        corridor_id = 0;
    else
        [in_corridor, theta_axis, axis_dist, corridor_id] = infer_corridor_axis_heading( ...
            x_pred(1:2), x_pred(3), cfg.corridor_segments_xyxy, cfg.corridor_half_width_m);
    end

    if in_corridor
        info.used_corridor_axis = true;
        info.corridor_id = corridor_id;
        info.axis_distance_m = axis_dist;
        sigma_theta_used = cfg.corridor_heading_std_rad;

        if cfg.use_axis_heading_only_in_corridor
            % Hard alignment to corridor axis.
            theta_target = theta_axis;
        else
            % Soft alignment: blend depends on lateral distance from axis.
            % Near centerline => stronger axis weight.
            blend_w = 1.0 - min(axis_dist / max(cfg.corridor_half_width_m, 1e-6), 1.0);
            z_mix = (1.0 - blend_w) * exp(1j * theta_motion) + blend_w * exp(1j * theta_axis);
            theta_target = angle(z_mix);
        end
    end

    % Step 3: scalar Kalman update on theta.
    % H_theta maps state to theta component.
    H_theta = [0, 0, 1, 0, 0];
    % Measurement variance on heading pseudo-measurement.
    R_theta = max(sigma_theta_used, 1e-3)^2;
    % Innovation must be wrapped because angles live on S1.
    innov_theta = wrap_angle_pi(theta_target - x_pred(3));
    S_theta = H_theta * P_pred * H_theta.' + R_theta;
    S_theta = max(S_theta, 1e-9);
    K_theta = (P_pred * H_theta.') / S_theta;

    x_out = x_pred + K_theta * innov_theta;
    x_out(3) = wrap_angle_pi(x_out(3));
    P_out = (I5 - K_theta * H_theta) * P_pred * (I5 - K_theta * H_theta).' + K_theta * R_theta * K_theta.';
    P_out = 0.5 * (P_out + P_out.');
    info.applied = true;
end

function [in_corridor, theta_axis, min_dist, best_id] = infer_corridor_axis_heading(xy, theta_ref, segments_xyxy, half_width_m)
%INFER_CORRIDOR_AXIS_HEADING
% Finds nearest corridor segment and returns:
% - in_corridor: true if distance to nearest centerline <= half_width_m
% - theta_axis : heading of that segment (direction selected vs theta_ref)
% - min_dist   : lateral distance to that segment
% - best_id    : selected segment index
    in_corridor = false;
    theta_axis = theta_ref;
    min_dist = Inf;
    best_id = 0;

    if isempty(segments_xyxy)
        return;
    end

    n_seg = size(segments_xyxy, 1);
    for i = 1:n_seg
        p1 = segments_xyxy(i, 1:2);
        p2 = segments_xyxy(i, 3:4);
        d_i = point_to_segment_distance_2d(xy, p1, p2);
        if d_i < min_dist
            min_dist = d_i;
            best_id = i;
            % Segment orientation (axis without direction sign disambiguation).
            theta_raw = atan2(p2(2) - p1(2), p2(1) - p1(1));
            % Pick axis direction (+/- pi) closest to current heading,
            % to avoid 180 deg flips.
            theta_axis = choose_axis_heading_direction(theta_raw, theta_ref);
        end
    end

    in_corridor = isfinite(min_dist) && (min_dist <= half_width_m);
end

function d = point_to_segment_distance_2d(p, a, b)
%POINT_TO_SEGMENT_DISTANCE_2D Euclidean distance from point p to segment ab.
    % Ensure consistent 1x2 shape to avoid implicit expansion issues.
    p = p(:).';
    a = a(:).';
    b = b(:).';

    ab = b - a;
    ab2 = dot(ab, ab);
    if ab2 <= 1e-12
        d = norm(p - a);
        return;
    end
    % Project point onto line, then clamp to segment [0,1].
    t = dot(p - a, ab) / ab2;
    t = min(max(t, 0.0), 1.0);
    proj = a + t * ab;
    d = norm(p - proj);
end

function theta_sel = choose_axis_heading_direction(theta_axis_raw, theta_ref)
%CHOOSE_AXIS_HEADING_DIRECTION
% A corridor axis has two equivalent directions: theta and theta+pi.
% We keep the one closer to current reference heading to preserve continuity.
    theta_a = wrap_angle_pi(theta_axis_raw);
    theta_b = wrap_angle_pi(theta_axis_raw + pi);
    if abs(wrap_angle_pi(theta_a - theta_ref)) <= abs(wrap_angle_pi(theta_b - theta_ref))
        theta_sel = theta_a;
    else
        theta_sel = theta_b;
    end
end

function ang = wrap_angle_pi(ang)
%WRAP_ANGLE_PI Wrap angle in radians to [-pi, pi].
    ang = mod(ang + pi, 2*pi) - pi;
end
