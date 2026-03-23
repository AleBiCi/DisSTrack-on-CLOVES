%% LOCALIZATION PROBLEM - MOVING UNICYCLE, EXTENDED KALMAN FILTER
% Consider a dynamic (moving) target and a fixed set of anchors.
% Estimate the evolving target position using a DYNAMIC ESTIMATOR (EKF)
% ! EKF is used since SYSTEM DYNAMICS ARE NONLINEAR
clear all;
clc;
close all;

%% Map Loading and Centroid definition
DEPTevb1000map = importfile("DEPT_evb1000_map.csv");
map = DEPTevb1000map(DEPTevb1000map.NodeId > 113 & DEPTevb1000map.NodeId < 154, :);
node_coords = split(map.Coordinates, ",");
map.lat = str2double(regexprep(node_coords(:,1), '[\\[\\]]', ''));
map.lon = str2double(regexprep(node_coords(:,2), '[\\[\\]]', ''));

% Centroid's definition
cluster_size = 3;
cluster_num = floor(size(map, 1) / cluster_size);
centroids = zeros(cluster_num, 2);
for i = 1:cluster_num
    idx = (i-1)*3 + (1:3);
    centroids(i, :) = [mean(map.lat(idx)), mean(map.lon(idx))];
end

%% Ground Truth trajectory definition
steps = 25; 
GT = [];
for i = 1:size(centroids, 1) - 1
    seg_x = linspace(centroids(i,1), centroids(i+1,1), steps)';
    seg_y = linspace(centroids(i,2), centroids(i+1,2), steps)';
    GT = [GT; [seg_x, seg_y]];
end
N_iters = size(GT, 1);

%% System definition
% Time step
dT = 1/30;

% Define system dynamics
fun = @(x, y, theta, vel, omega) [x; y; theta] + [vel; vel; omega] .* [cos(theta); sin(theta); 1] .* dT;

% Define the system Jacobian
A = @(x, y, theta, vel, omega) [
    1,  0,  -vel*sin(theta)*dT; ...
    0,  1,  vel*cos(theta)*dT;  ...
    0,  0,  1
];

G = eye(3);

nu = [0; 0; 0];

noise_std = 0.1;

Q = 0.05 * eye(3);

% # measurements
k = 300;

% Define anchors
anchors = [map.lat, map.lon];
n_anchors = size(anchors, 1);
state = [GT(1,1); GT(1,2); 0];

% Calculate from anchors to target
distances = sqrt(sum((anchors - state(1:2)').^2, 2));

% Add noise to distances
dist_noisy = distances + 0.1 * randn(n_anchors, 1);

% Init first position used to initialize EKF (x_0 and P_0)
[H, z, C] = trilateration(anchors, dist_noisy, noise_std);
P = (H'* C^-1 * H)^-1;
x_ls = P * H' * C^-1 * z; % same as for LS
%% Init the EKF

x_values = zeros(N_iters, 3);
x_values(1,:) = [x_ls', 0];
P_values = [P, zeros(2, 1); zeros(1, 3)];
P_values(3,3) = 0.1;
P = P_values;

% Estimated (predicted in this case since we can't measure and update them
% directly) values of theta
theta_values = x_values(:,3);

% Initialize the plot
figure;
hold on;
plot(anchors(:,1), anchors(:,2), 'ro', 'MarkerSize', 10, 'DisplayName', 'Anchors');
% Initialize handles for real and estimated positions
line_handle = plot(NaN, NaN, 'bo-', 'MarkerSize', 5, 'DisplayName', 'Real Trajectory');
line_handle_est = plot(NaN, NaN, 'g+', 'MarkerSize', 10, 'DisplayName', 'Estimated Position');
% Initialize the handles for the heading vectors
% The '0' scale factor ensures the vector length is exactly the U/V components (1 * cos(theta), 1 * sin(theta))
quiver_real_handle = quiver(NaN, NaN, NaN, NaN, 0, 'Color', 'b', 'LineWidth', 0.3, 'MaxHeadSize', 0.3, 'DisplayName', 'Real Heading');
quiver_est_handle = quiver(NaN, NaN, NaN, NaN, 0, 'Color', 'g', 'LineWidth', 0.3, 'MaxHeadSize', 0.3, 'DisplayName', 'Estimated Heading');
title('Real-Time Dynamical System Trajectory');
xlabel('x');
ylabel('y');
grid on;
legend;

%Visualize the centroids in the map
plot(centroids(:,1), centroids(:,2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);

% Real-time update
x_data = [];
y_data = [];
theta_data = [];
vel = 3.0;
omega = -0.9;

N_iters = size(GT, 1);
trace_P = zeros(N_iters,1);
trace_P(1) = trace(P);

%% Real time simulation
x_data = GT(1,1); y_data = GT(1,2); theta_data = 0;

for k = 2:N_iters
    
    % Real time simulation
    dx = GT(k,1) - GT(k-1,1);
    dy = GT(k,2) - GT(k-1,2);
    vel_inst = sqrt(dx^2 + dy^2) / dT;

    real_theta = atan2(dy, dx);
    if k > 2
        prev_theta = atan2(GT(k-1,2)-GT(k-2,2), GT(k-1,1)-GT(k-2,1));
        omega_inst = (real_theta - prev_theta) / dT; 
    else
        omega_inst = 0;
    end

    % PREDICTION (based on the previous estimation and the unicycle model)
    [x_pred, P_pred] = predict_step([x_values(k-1, 1), x_values(k-1, 2), x_values(k-1, 3), vel_inst, omega_inst], P, A, G, Q, fun);

    % DYNAMIC CLUSTERING (Selection of the 3 nearest anchors)
    % Loading - compute the distances - select the anchors - activate them
    all_anchors = [map.lat, map.lon]; 
    dist_to_all = sqrt(sum((all_anchors - x_pred(1:2)').^2, 2)); 
    [~, sorted_idx] = sort(dist_to_all); 

    % Search the 3 non align best anchors
    best_anchors = false;
    search_idx = 3; % start from the nearest 3
    
    while ~best_anchors && search_idx <= length(sorted_idx)
        candidate_idx = sorted_idx(1:search_idx);
        % Try to found the 3 nearest anchor not aligned
        current_selection = sorted_idx([1, 2, search_idx]); 
        active_anchors = all_anchors(current_selection, :);
        
        % Alignment check: compute the standard dev of X and Y
        std_x = std(active_anchors(:,1));
        std_y = std(active_anchors(:,2));
        
        % If the std dev is too low then change 
        threshold_align = 0.5; % meters
        if std_x > threshold_align && std_y > threshold_align
            best_anchors = true;
        else
            search_idx = search_idx + 1; % Use another anchors better nonaligned
        end
        
        % Security check
        if search_idx > 10 
            active_anchors = all_anchors(sorted_idx(1:3), :); 
            break; 
        end
    end

    % REAL MEASUREMENT SIMULATIONS (Robot's dynamic)
    % Compute the real angle based on the GT for the simulation
    state = [GT(k,1); GT(k,2); real_theta];
    
    % Simulation of the noise. It increase with the distance from the sensor
    dist_real_anchors = sqrt(sum((active_anchors - state(1:2)').^2, 2));
    noise_vector = noise_std + 0.02 * dist_real_anchors;  
    distances_sim = dist_real_anchors + noise_vector .* randn(3, 1);

    % OUTLIER REJECTION
    dist_expected = sqrt(sum((active_anchors - x_pred(1:2)').^2, 2));
    
    % Parameters
    k_min = 1.5; % Strict when is slow
    alpha = 0.5; % How much the tolerance grows at each m/s of velocity

    % Dynamic multiplier
    k_dynamic = k_min + alpha * vel_inst;

    % Dynamic threshold
    threshold = k_dynamic * sqrt(trace(P_pred));% We use the uncertainty of the filter
    residual = abs(distances_sim - dist_expected);
    is_valid = residual < threshold;
    
    % If we have less than 3 good measure we skip the update
    if sum(is_valid) >= 3 
        % Good measure --> UPDATE
        [H_short, z, C] = trilateration(active_anchors(is_valid,:), distances_sim(is_valid), noise_vector(is_valid));
        H_full = [H_short, zeros(size(H_short, 1), 1)]; 
        [x_values(k,:), P] = update_step(x_pred, P_pred, z, H_full, C);
    else
        % Bad measure --> Rejected
        x_values(k,:) = x_pred';
        P = P_pred; 
        fprintf('Outlier rilevato al passo %d: Update saltato.\n', k);
    end


    % UPDATE THE PLOT
    % save the real data
    x_data = [x_data, state(1)];
    y_data = [y_data, state(2)];
    
    % Find the vector for heading (Real and Estimate)
    mag = 0.5;
    u_real = mag * cos(state(3));
    v_real = mag * sin(state(3));
    u_est = mag * cos(x_values(k, 3));
    v_est = mag * sin(x_values(k, 3));
    
    % Update Position Markers 
    set(line_handle, 'XData', x_data, 'YData', y_data);
    set(line_handle_est, 'XData', x_values(1:k,1)', 'YData', x_values(1:k,2)');
    set(quiver_real_handle, 'XData', state(1), 'YData', state(2), 'UData', u_real, 'VData', v_real);
    set(quiver_est_handle, 'XData', x_values(k, 1), 'YData', x_values(k, 2), 'UData', u_est, 'VData', v_est);
    
    % Active Anchors
    hActive = plot(active_anchors(:,1), active_anchors(:,2), 'go', 'MarkerSize', 12, 'LineWidth', 2);
    
    drawnow;
    delete(hActive); % Clean the next frame
    
    % Save Trace and Covariance
    trace_P(k) = trace(P);
end

% Plot the results of the estimation
figure;
plot(trace_P)
title('Trace of the covariance matrix');
xlabel('Time step');
ylabel('Trace of the covariance matrix');

%% Performance Analysis - RMSE
% Compute the position error at each time step
% x_values(1:k, 1:2) are the estimation, GT(1:k, :) is the GT
errors_xy = sqrt(sum((x_values(1:N_iters, 1:2) - GT(1:N_iters, :)).^2, 2));

% RMSE global error
rmse_total = sqrt(mean(errors_xy.^2));

fprintf('Performance Analysis\n');
fprintf('RMSE Total Position: %.4f meters\n', rmse_total);
fprintf('Maximum error: %.4f meters\n', max(errors_xy));

% Plot the RMSE
figure;
plot(errors_xy, 'LineWidth', 0.5);
grid on;
title(['Position Error over Time (RMSE: ', num2str(rmse_total, 3), ' m)']);
xlabel('Time step');
ylabel('Error [m]');


%% Trilateration function for 1 time step

function [H, z, C] = trilateration(anchors, distances, noise_std)
    % noise_std is a vector with all the std dev for all the anchors
    % => to get variances do .^2

    n = size(anchors,1);
    % if noise_std is given as single value (same for all the anchors) then
    % vectorize it
    if size(noise_std, 1) == 1
        noise_std_vectorized = ones(n).*noise_std;
    else
        noise_std_vectorized = noise_std;
    end

    % Init matrices
    H = zeros(n-1, 2);  % [n-1 x 2]
    z = zeros(n-1, 1);  % measurements [n-1 x 1]
    C = zeros(n-1);     % covariance matrix [n-1 x n-1]

    % Iterate over all anchors
    for i=1:n-1
        % Fill matrices (same as solution without uncertainties)
        H(i, :) = [-2*anchors(i, 1) + 2*anchors(i+1, 1), -2*anchors(i, 2) + 2*anchors(i+1, 2)];
        z(i) = distances(i)^2 - distances(i+1)^2 - anchors(i, 1)^2 + anchors(i+1, 1)^2 - anchors(i, 2)^2 + anchors(i+1, 2)^2;

        % Fill C row by row
        C(i,i) = 4*distances(i)^2*noise_std_vectorized(i)^2 + 4*distances(i+1)^2*noise_std_vectorized(i+1)^2; % Diagonal elements
        if i == 1
            % only for first element of first row
            C(i, i+1) = - 4*distances(i+1)^2*noise_std_vectorized(i+1)^2;
        elseif i == n-1
            % only for last element of last row
            C(i, i-1) = - 4*distances(i-1)^2*noise_std_vectorized(i-1)^2;
        else
            % if C is bigger than 2x2, this fills both the element on the
            % left and on the right of the diagonal element
            C(i, i+1) = - 4*distances(i+1)^2*noise_std_vectorized(i+1)^2;
            C(i, i-1) = - 4*distances(i-1)^2*noise_std_vectorized(i-1)^2;
        end
    end
end
%% EKF Prediction and Update functions

% Prediction step
function [x_pred, P_pred] = predict_step(x, P, A, G, Q, fun)
    x_pred = fun(x(1), x(2), x(3), x(4), x(5)); % Assume x contains [x,y,theta,vel,omega]
    A_c = A(x(1), x(2), x(3), x(4), x(5));
    P_pred = A_c * P * A_c' + G * Q * G';
end

% Update step
function [x_k_1, P_k_1] = update_step(x_k, P_k, z_k_1, H_k_1, C_new)
    S_k_1 = H_k_1 * P_k * H_k_1' + C_new;
    K_k_1 = P_k * H_k_1' * S_k_1^-1;
    x_k_1 = x_k + K_k_1*(z_k_1 - H_k_1*x_k);
    P_k_1 = (eye(3) - K_k_1*H_k_1) * P_k;
end     