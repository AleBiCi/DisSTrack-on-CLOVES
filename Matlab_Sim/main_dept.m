%% LOCALIZATION PROBLEM - UNICYCLE, RANGE-ONLY EKF
% Real-time: reads full UWB rounds from the EVB1000 tag via USB serial.
% This revision implements:
%   - System definition (unicycle EKF model)
%   - Weighted trilateration with WLS + P0 init
clearvars; clc; close all;

%% Variables
tracking_node_ids = [108, 113:119, 121:154];
default_noise = 0.07;
z_fixed_m = 1.30; % [m] tag height used in range model (set as needed)

% Room-constraint flags (hooks kept explicit for later pipeline steps).
enable_room_constraint = true;
room_model_path = "room_constraint_model_projective.mat";
room_constraint_on_init = true;
room_constraint_on_predict = false;
room_constraint_on_update = true;
room_projection_threshold_m = 0.70;

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
sigma_omega_process = 0.18;   % [rad/s]

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
cfg_init.max_init_attempts = 30;                  % max serial rounds to find a valid initialization
cfg_init.initial_heading_std_rad = deg2rad(75);   % std dev of initial heading uncertainty
cfg_init.initial_speed_std_mps = 0.70;            % std dev of initial linear speed
cfg_init.initial_yaw_rate_std_radps = 0.90;       % std dev of initial yaw rate
cfg_init.min_xy_variance_m2 = 0.20^2;             % minimum variance floor on x/y at initialization
cfg_init.range_variance_distance_gain = 2e-5;     % k in sigma^2(d) = sigma0^2 + k*d^2

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
max_trail_points = 50; % approx. 2 points per second ==> plot 25s of estimates
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
colormap(ax_main, parula(256));
cb_anchor_var = colorbar(ax_main, 'eastoutside');
cb_anchor_var.Label.String = 'Varianza ancore usate [m^2]';
xlabel('X [m]'); ylabel('Y [m]');
title('Real-Time UWB EKF Tracking  |  single-round initialization');
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

% INITIALIZATION ROUND
[x0, P0, init_info] = initialize_state_weighted_single_round( ...
    s, map, addr_short, noise_map, bias_map, default_noise, z_fixed_m, cfg_init, ...
    room_constraint, room_constraint_on_init, room_projection_threshold_m);

fprintf('[init] single-round init: r=%d raw/used=%d/%d mae=%.2fm\n', ...
    init_info.round_id, init_info.raw_count, init_info.used_count, init_info.mae_m);
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
heading_prior_cfg.corridor_half_width_m = 1.40;            % [m]
% If true: snap directly to axis heading.
% If false: blend motion heading and axis heading based on lateral distance.
heading_prior_cfg.use_axis_heading_only_in_corridor = true;
% Corridor centerlines as [x1 y1 x2 y2] segments (blue area).
heading_prior_cfg.corridor_segments_xyxy = [ ...
    133.6, 26.1, 188.6, 26.1; ...
    133.6, 10.9, 133.6, 26.1; ...
    133.6, 10.9, 188.6, 10.9];
% Atrium polygon (green area): inside this region, do NOT apply corridor axis forcing (keep only motion-derived heading prior).
heading_prior_cfg.disable_corridor_prior_in_atrium = true;
heading_prior_cfg.atrium_polygon_xy = [ ...
    181.0, 24.0; ...
    189.4, 24.0; ...
    189.4, 12.8; ...
    181.0, 12.8];

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

    % EKF range update.
    % A round can contain one or more anchor ranges. With one anchor, the
    % update only constrains the state along that anchor's range direction;
    % with multiple anchors, all valid ranges are fused in the same EKF step.
    if used_count >= 1 && ~isempty(rr.anchors_xyz) && size(rr.anchors_xyz, 1) == length(rr.distances_m)
        z_k = rr.distances_m(:);
        h_k = h_range(x_pred, rr.anchors_xyz);
        H_k = H_range(x_pred, rr.anchors_xyz);
        % CRITICAL CHECK: Ensure H_k dimensions match [Measurements x States]
        if size(H_k, 1) == length(z_k) && size(H_k, 2) == 5
            % Measurement covariance. sigma2_used comes from the per-anchor
            % noise map plus the distance-dependent variance term. The
            % diagonal form assumes independent range errors between anchors.
            R_k = diag(sigma2_used);
            innov_k = z_k - h_k;
            % Innovation covariance. Symmetrization removes tiny numerical
            % asymmetries before solving for the Kalman gain.
            S_k = H_k * P_pred * H_k.' + R_k;
            S_k = 0.5 * (S_k + S_k.');
        else
            % If the measurement Jacobian is inconsistent, skip the range
            % correction and keep the prediction for this round.
            used_count = 0;
        end

        if used_count == 1
            % Scalar update for a single range measurement. This avoids a
            % matrix solve and guards against division by an almost-zero S.
            S_scalar = S_k(1,1);
            S_scalar = max(S_scalar, 1e-9);
            K_k = (P_pred * H_k.') / S_scalar;
        else
            % Multi-range update. Poor geometry or nearly redundant anchors can
            % make S_k ill-conditioned, so fall back to a pseudoinverse.
            if rcond(S_k) < 1e-10
                S_inv = pinv(S_k);
                K_k = P_pred * H_k.' * S_inv;
            else
                K_k = (P_pred * H_k.') / S_k;
            end
        end

        x_upd = x_pred + K_k * innov_k;
        % Joseph-form covariance update, used for better numerical stability
        % and to preserve positive semidefiniteness in finite precision.
        P_upd = (I5 - K_k * H_k) * P_pred * (I5 - K_k * H_k).' + K_k * R_k * K_k.';
        mode_str = "update";
    else
        % No usable range data in this round: run prediction-only.
        x_upd = x_pred;
        P_upd = P_pred;
    end

    % Room/map constraint.
    % This is a geometric post-filter applied after the EKF range update. It
    % is not another stochastic measurement in R_k; instead, it enforces that
    % the estimated position remains inside the free-space model.
    if room_constraint_on_update && ~isempty(room_constraint)
        [xy_upd_room, info_upd_room] = apply_room_constraint(room_constraint, x_upd(1:2).', ...
            'projection_threshold_m', room_projection_threshold_m, 'fallback_pose', x_pred(1:2).', 'label', 'main_new_update');
        if strcmp(info_upd_room.action, 'project')
            % Close to the valid map: snap x/y to the nearest feasible point
            % and inflate the planar covariance to acknowledge the correction.
            x_upd(1:2) = xy_upd_room(:);
            P_upd(1:2,1:2) = P_upd(1:2,1:2) + (0.10^2) * eye(2);
            mode_str = "map project";
            note_str = "project";
            fprintf('[room] round %d update projected (dist=%.2f m, thr=%.2f m)\n', ...
                rr.round_id, info_upd_room.distance_to_free_m, room_projection_threshold_m);
        elseif strcmp(info_upd_room.action, 'reject')
            % Too far outside the valid map: discard the measurement-updated
            % pose and keep the prediction as the safest state for this round.
            x_upd = x_pred;
            P_upd = P_pred;
            mode_str = "map reject";
            note_str = "reject";
            fprintf('[room] round %d update rejected (dist=%.2f m > thr=%.2f m), using prediction\n', ...
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
    if isnan(rr.round_id)
        r_print = -1;
    else
        r_print = rr.round_id;
    end
    mode_char = char(mode_str);

    title(sprintf('Real-Time UWB EKF | mode=%s | r=%d | pos=(%.2f, %.2f)', upper(mode_char), r_print, x_est(1), x_est(2)));
    set(status_text, 'String', sprintf('k=%d | round=%d | mode=%s | note=%s | anchors=%d', ...
        k_loop, r_print, mode_char, char(note_str), used_count));
    drawnow limitrate;
    gif_recorder = map_gif_recorder('capture', gif_recorder, map_fig);

    fprintf('[track] r=%d k=%d mode=%s note=%s raw/used=%d/%d pos=(%.2f,%.2f) v=%.2f w=%.2f cov=%.3f\n', ...
        r_print, k_loop, mode_char, char(note_str), rr.raw_count, used_count, ...
        x_est(1), x_est(2), x_est(4), x_est(5), cov_xy_trace);
end

%% Save recorded map GIF
map_gif_recorder('prompt_save', gif_recorder);

%% Local functions
function H = build_range_jacobian_unicycle(xk, anchors_xyz, z_fixed_m)
%BUILD_RANGE_JACOBIAN_UNICYCLE Linearized range model wrt [px py theta v omega].
    n = size(anchors_xyz, 1);
    H = zeros(n, 5);
    dx = xk(1) - anchors_xyz(:,1);
    dy = xk(2) - anchors_xyz(:,2);
    dz = z_fixed_m - anchors_xyz(:,3);
    rr = sqrt(dx.^2 + dy.^2 + dz.^2);
    % Avoid division by zero if the estimate is exactly at an anchor.
    rr = max(rr, 1e-6);
    H(:,1) = dx ./ rr;   % d(range)/d(px)
    H(:,2) = dy ./ rr;   % d(range)/d(py)
    % theta, v, omega have no direct range derivative.
end

function [x0, P0, info] = initialize_state_weighted_single_round( ...
    s, map, addr_short, noise_map, bias_map, default_noise, z_fixed_m, cfg_init, ...
    room_constraint, use_room_constraint, room_projection_threshold_m)
%INITIALIZE_STATE_WEIGHTED_SINGLE_ROUND Build x0/P0 from the first usable round.
    ok = false;
    rr = struct('round_id', NaN, 'raw_count', 0, 'used_count', 0, ...
        'anchors_xyz', zeros(0,3), 'distances_m', zeros(0,1), 'std_m', zeros(0,1));
    xy = [NaN; NaN];
    Pxy = [];
    mae = NaN;

    for attempt = 1:cfg_init.max_init_attempts
        % Serial data can arrive incomplete; retry until WLS returns a finite pose.
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
        % The initializer can start outside free space; project only if close.
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

    % With a single round there is no reliable motion-based heading yet.
    theta0 = 0;
    sigma_theta = cfg_init.initial_heading_std_rad;

    if isempty(Pxy) || any(~isfinite(Pxy(:)))
        % WLS can fail to produce covariance with poor anchor geometry.
        Pxy = cfg_init.min_xy_variance_m2 * eye(2);
    end
    % Symmetrize and add a floor so the EKF does not start overconfident.
    Pxy0 = 0.5 * (Pxy + Pxy.') + cfg_init.min_xy_variance_m2 * eye(2);
    Pxy0 = 0.5 * (Pxy0 + Pxy0.');

    x0 = [xy(1); xy(2); theta0; 0; 0];
    P0 = blkdiag(Pxy0, sigma_theta^2, cfg_init.initial_speed_std_mps^2, cfg_init.initial_yaw_rate_std_radps^2);

    info = struct();
    info.round_id = rr.round_id;
    info.raw_count = rr.raw_count;
    info.used_count = rr.used_count;
    info.mae_m = mae;
end

function rr = collect_round_measurements_all( ...
    s, map, addr_short, noise_map, bias_map, default_noise, stop_handle)
%COLLECT_ROUND_MEASUREMENTS_ALL Return one complete round with all anchors.
    if nargin < 7
        stop_handle = [];
    end

    while true
        if ~isempty(stop_handle) && ~isgraphics(stop_handle)
            % The plot was closed while blocking on serial input.
            rr = [];
            return;
        end

        [round_meas, round_id] = read_single_round(s, map, addr_short, noise_map, bias_map, default_noise);
        if isempty(round_meas)
            % No complete measurement yet; avoid spinning at full CPU.
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
    
    try
        % Nonblocking guard: return if the serial buffer has no full line yet.
        if s.NumBytesAvailable == 0
            return; 
        end
        raw_line = readline(s);
    catch
        fprintf('[serial] Warning: low-level transport error (SceneNode).\n');
        return;
    end

    if isempty(raw_line) || strlength(raw_line) == 0
        % Ignore keepalive/blank lines before entering the round parser.
        return;
    end

    while true
        line = strtrim(readline(s));
        [is_meas, meas] = parse_ranging_meas(line);

        if is_meas
            if isnan(round_id)
                round_id = meas.round_id;
            elseif meas.round_id ~= round_id
                % New round before a summary: drop the partial old round.
                round_id = meas.round_id;
                round_meas = empty_round_meas();
            end

            idx_map = find(strcmpi(addr_short, meas.addr), 1);
            if isempty(idx_map)
                % Measurement from a node not present in the selected map.
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

            % Store bias-corrected range plus per-anchor std for R_k.
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
                % Summary marks the end of this round in both log formats.
                return;
            end
        end
    end
end

function meas = empty_round_meas()
%EMPTY_ROUND_MEAS Typed empty struct for easy end+1 appends.
    meas = struct('addr', {}, 'dist_m', {}, 'noise', {}, 'x', {}, 'y', {}, 'z', {});
end

function tf = should_replace_measurement(new_meas, old_meas)
%SHOULD_REPLACE_MEASUREMENT Keep the newest duplicate anchor sample.
    tf = true;
end

function [is_meas, meas] = parse_ranging_meas(line)
%PARSE_RANGING_MEAS Parse one "RANGING MEAS" line.
    expr = ['RANGING MEAS \[(?<round>\d+)\]\s+\[[\w:]+->(?<addr>[\w:]+)\]\s+' ...
        '(?<dist>\d+)\s+mm'];
    tok = regexp(char(line), expr, 'names');
    if isempty(tok)
        % Caller treats non-measurement lines as possible round summaries.
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
%PARSE_ROUND_SUMMARY Parse either supported round-boundary format.
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
%WEIGHTED_TRILATERATION_WLS_ALL Estimate tag xy from one ranging round.
% Noonlinear WLS minimizes sum_i ((d_i - h_i(x,y)) / sigma_i)^2 with Gauss-Newton.

    % Default to an invalid result; only set ok=true after all outputs are finite.
    xy = [NaN; NaN];
    Pxy = [];
    mae = NaN;
    ok = false;

    % Step 0: require at least one anchor before building the WLS system.
    n = size(anchors_xyz, 1);
    if n < 1
        % No anchors means no geometric information.
        return;
    end

    % Step 1: build the diagonal inverse-variance weight matrix.
    % sigma_i^2 combines the anchor baseline noise with a distance-dependent
    % term, so noisy/far anchors receive a smaller weight in the fit.
    sigma2 = range_variance_model(std_m(:), distances_m(:), distance_variance_gain);
    W = diag(1 ./ sigma2);

    % Step 2: choose the starting point for the nonlinear solve.
    if isempty(x0) || any(~isfinite(x0))
        % Without a finite prior, use a stable inverse-variance anchor centroid.
        weights = 1 ./ sigma2;
        weights = weights / sum(weights);
        xk = [sum(weights .* anchors_xyz(:,1)); sum(weights .* anchors_xyz(:,2))];
    else
        % With a valid prior, warm-start WLS from that previous/best estimate.
        xk = x0(:);
    end

    % Step 3: iterate Gauss-Newton at most 15 times on the nonlinear range residuals.
    lambda = 1e-4; % Light damping keeps the normal equations usable with weak geometry.
    for it = 1:15
        % Offsets from each anchor to the current xy estimate; dz is fixed.
        dx = xk(1) - anchors_xyz(:,1);
        dy = xk(2) - anchors_xyz(:,2);
        dz = z_fixed_m - anchors_xyz(:,3);

        % Predicted ranges h_i(xk). The floor avoids division by zero in H.
        hk = sqrt(dx.^2 + dy.^2 + dz.^2);
        hk = max(hk, 1e-6);

        % Residuals and Jacobian of h_i with respect to [x,y].
        % Positive residual means the measured range is longer than predicted.
        res = distances_m(:) - hk;
        H = [dx ./ hk, dy ./ hk];

        % Weighted normal equations for the Gauss-Newton correction step.
        N = H.' * W * H + lambda * eye(2); % Fisher information matrix
        g = H.' * W * res; % Gradient of N

        if rcond(N) < 1e-10
            % Rank-poor geometry: use the damped pseudo-inverse step.
            step = pinv(N) * g;
        else
            step = N \ g;
        end

        % Apply the local linearized correction.
        xk = xk + step;
        if norm(step) < 1e-4
            % Sub-millimetric update: the solution is stable enough to stop.
            break;
        end
    end

    % Step 4: recompute residuals/Jacobian at the final xy estimate.
    dx = xk(1) - anchors_xyz(:,1);
    dy = xk(2) - anchors_xyz(:,2);
    dz = z_fixed_m - anchors_xyz(:,3);
    hk = sqrt(dx.^2 + dy.^2 + dz.^2);
    hk = max(hk, 1e-6);
    res = distances_m(:) - hk;
    H = [dx ./ hk, dy ./ hk];

    % Step 5: approximate local xy covariance from the inverse information (aka Pxy = N^-1).
    N = H.' * W * H + lambda * eye(2);
    if rcond(N) < 1e-10
        % Use pinv when the anchor layout is nearly rank deficient.
        Pxy = pinv(N);
    else
        Pxy = inv(N);
    end

    % Step 6: return the estimate, a residual quality score, and validity.
    xy = xk;
    mae = mean(abs(res), 'omitnan');
    ok = all(isfinite([xy; mae])) && all(isfinite(Pxy(:)));
end

function sigma2 = range_variance_model(sigma0_m, distance_m, distance_variance_gain)
%RANGE_VARIANCE_MODEL Per-anchor distance-dependent variance model:
% sigma_i^2(d) = sigma0_i^2 + k * d_i^2
    % Floors prevent zero-variance anchors from dominating EKF/WLS.
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
        % Plot NaNs instead of throwing if covariance is unavailable.
        return;
    end

    [V, D] = eig(P_sym);
    eigvals = diag(D);
    % Tiny negative eigenvalues can appear from roundoff.
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
    % Keep the patch a cone instead of wrapping into a full disk.
    half_angle = min(half_angle, pi - 1e-3);

    arc_angles = linspace(theta - half_angle, theta + half_angle, max(6, n_arc_pts));
    arc_x = mu_xy(1) + cone_len_m * cos(arc_angles);
    arc_y = mu_xy(2) + cone_len_m * sin(arc_angles);

    x_cone = [mu_xy(1), arc_x, mu_xy(1)];
    y_cone = [mu_xy(2), arc_y, mu_xy(2)];
end

function [x_out, P_out, info] = apply_heading_prior_unicycle(x_pred, P_pred, x_prev_est, I5, cfg)
%APPLY_HEADING_PRIOR_UNICYCLE Pseudo-measure theta from motion/corridor cues.
    x_out = x_pred;
    P_out = P_pred;
    info = struct('applied', false, 'used_corridor_axis', false, ...
        'dist_moved_m', 0.0, 'corridor_id', 0, 'axis_distance_m', Inf, ...
        'in_atrium', false);

    if ~cfg.enabled
        % Feature flag lets the EKF run as pure range-only.
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
        % Atrium/open areas should not force a corridor-aligned heading.
        poly = cfg.atrium_polygon_xy;
        in_atrium = inpolygon(x_pred(1), x_pred(2), poly(:,1), poly(:,2));
    end
    info.in_atrium = in_atrium;

    if in_atrium
        % Keep the motion-derived prior only.
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
    % Innovation must be wrapped because angles live on a circle.
    innov_theta = wrap_angle_pi(theta_target - x_pred(3));
    S_theta = H_theta * P_pred * H_theta.' + R_theta;
    % Guard the scalar division if P/R tuning becomes too aggressive.
    S_theta = max(S_theta, 1e-9);
    K_theta = (P_pred * H_theta.') / S_theta;

    x_out = x_pred + K_theta * innov_theta;
    x_out(3) = wrap_angle_pi(x_out(3));
    P_out = (I5 - K_theta * H_theta) * P_pred * (I5 - K_theta * H_theta).' + K_theta * R_theta * K_theta.';
    P_out = 0.5 * (P_out + P_out.');
    info.applied = true;
end

function [in_corridor, theta_axis, min_dist, best_id] = infer_corridor_axis_heading(xy, theta_ref, segments_xyxy, half_width_m)
%INFER_CORRIDOR_AXIS_HEADING Nearest corridor centerline and aligned heading.
    in_corridor = false;
    theta_axis = theta_ref;
    min_dist = Inf;
    best_id = 0;

    if isempty(segments_xyxy)
        % No corridor model configured.
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
        % Degenerate segment: treat it as a point.
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
