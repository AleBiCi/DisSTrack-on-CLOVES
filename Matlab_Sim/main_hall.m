%% LOCALIZATION PROBLEM - UNICYCLE, RANGE-ONLY EKF
% Real-time: reads full UWB rounds from the EVB1000 tag via USB serial.
% This revision implements:
%   - System definition (unicycle EKF model)
%   - Weighted trilateration with WLS + P0 init
clearvars; clc; close all;

%% Variables
tracking_node_ids = [50:58, 61:65, 70:77];
default_noise = 0.07;
z_fixed_m = 1.30; % [m] tag height used in range model (set as needed)

% Room-constraint flags (hooks kept explicit for later pipeline steps).
enable_room_constraint = true;
room_model_path = "room_constraint_model_hall.mat";
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
HALLevb1000map = importfile("HALL-A_evb1000_map.csv");
map = HALLevb1000map(ismember(HALLevb1000map.NodeId, tracking_node_ids), :);
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
if isfile("variance_per_node_hall.csv")
    var_table = readtable("variance_per_node_hall.csv");
    noise_map = containers.Map(var_table.node_id, var_table.pooled_std_m);
    bias_map = containers.Map('KeyType', 'double', 'ValueType', 'double');
    BIAS_THRESHOLD_M = 0.5; % ignore anchor biases larger than 50 cm
    % Hall-specific list: keep empty until hall variance analysis marks bad nodes.
    bad_bias_nodes = [];
    fprintf("[init] Hall bad_bias_nodes exclusions: %d\n", numel(bad_bias_nodes));
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
    fprintf("[init] variance_per_node_hall.csv not found, default use %.3f m\n", default_noise);
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
sigma_v_process = 0.20;       % [m/s]
sigma_omega_process = 0.35;   % [rad/s]

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
gif_cfg.output_basename = "main_hall_map_recording";
gif_cfg.output_dir = fullfile(project_root, 'recordings', 'hall');

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

    % Pseudo-measurement: heading from position displacement 
    dp = x_pred(1:2) - x_prev_est(1:2);
    dist_moved = norm(dp);
    if dist_moved > dist_moved_thresh  % only inject when moved > 50 cm
        theta_pseudo = atan2(dp(2), dp(1));
        H_theta = [0, 0, 1, 0, 0];
        innov_theta = wrap_angle_pi(theta_pseudo - x_pred(3));
        R_theta = (sigma_theta)^2;  % tune: larger = trust position-derived heading less
        S_theta = H_theta * P_pred * H_theta.' + R_theta;
        K_theta = (P_pred * H_theta.') / S_theta;
        x_pred = x_pred + K_theta * innov_theta;
        x_pred(3) = wrap_angle_pi(x_pred(3));
        P_pred = (I5 - K_theta * H_theta) * P_pred * (I5 - K_theta * H_theta).' + K_theta * R_theta * K_theta.';
    end
    % end pseudo-measurement 

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
            mode_str = "map_project";
            note_str = "project";
            fprintf('[room] round %d update projected (dist=%.2f m, thr=%.2f m)\n', ...
                rr.round_id, info_upd_room.distance_to_free_m, room_projection_threshold_m);
        elseif strcmp(info_upd_room.action, 'reject')
            x_upd = x_pred;
            P_upd = P_pred;
            mode_str = "map_reject";
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
    set(quiv_est, 'XData', traj_x, 'YData', traj_y, 'UData', traj_u, 'VData', traj_v);
    set(h_used_anchors, 'XData', rr.anchors_xyz(:,1), 'YData', rr.anchors_xyz(:,2), 'CData', sigma2_used);
    anchor_var_min_m2 = min(anchor_var_min_m2, min(sigma2_used));
    anchor_var_max_m2 = max(anchor_var_max_m2, max(sigma2_used));
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

    for attempt = 1:cfg_init.max_init_attempts
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

    % With a single round there is no reliable motion-based heading yet.
    theta0 = 0;
    sigma_theta = cfg_init.initial_heading_std_rad;

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

function ang = wrap_angle_pi(ang)
%WRAP_ANGLE_PI Wrap angle in radians to [-pi, pi].
    ang = mod(ang + pi, 2*pi) - pi;
end
