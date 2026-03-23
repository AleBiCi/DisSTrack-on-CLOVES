%% 
%     "DEPT" NODE MAP VISUALIZATION w/ given coordinates
%

clear all; clc;

%% Import node coords and IDs - EVB1000 only, DEPT zone, floor 3 Povo 1, main structure (no bridge)

DEPTevb1000map = importfile("DEPT_evb1000_map.csv");

map = DEPTevb1000map(DEPTevb1000map.NodeId > 113 & DEPTevb1000map.NodeId < 154, :);

node_coords = split(map.Coordinates, ",");

map.lat = str2double(regexprep(node_coords(:,1), '[\[\]]', ''));
map.lon = str2double(regexprep(node_coords(:,2), '[\[\]]', ''));

node_num = size(map, 1)

%% Create clusters and assign groups of nodes to them
% IDEA:
%   - each cluster contains 3 nodes
%   - the cluster area is defined as a circle w/ certain radius centered in
%       the centroid of the shape defined by the 3 nodes (vertexes)
%   - add "cluster" entry in the 'map' table
%   - when tag exits the cluster area stop calling localization routine for
%       that cluster

cluster_size = 3;
cluster_num = floor(node_num / cluster_size);

centroids = zeros(cluster_num, 2);  % init (x,y) coordinates of cluster's centroids

for i=1:cluster_num
    idx = (i-1)*cluster_size + (1:cluster_size);  % indices of nodes in cluster i
    
    % calculate centroids coords
    centroids(i,1) = mean(map.lat(idx));
    centroids(i,2) = mean(map.lon(idx));

    % Assign nodes (by NodeID) to clusters
    nodes_in_cluster(i,1:cluster_size) = map.NodeId(idx);

    % Cluster area radii as the maximum distance between a node and the
    % centroid + epsilon (1.0m)
    cluster_radii(i) = max(vecnorm([map.lat(idx), map.lon(idx)] - centroids(i,:), 2, 2)) + 1.0;
end

% Create clusters table
clusters = table((1:cluster_num)', nodes_in_cluster, centroids, cluster_radii', lines(cluster_num), 'VariableNames', ["ClusterID", "NodeID", "Centroid", "Radius", "Color"]);

%% Draw the map w/ nodes and cluster areas

% Turn Data Cursor Mode on, but keep figure callback free
dcm = datacursormode(gcf);
dcm.Enable = 'off';  % Disable default interaction

figure(1), clf, axis equal, grid on, grid minor, hold on;

% Plot nodes and centroids
for i = 1:cluster_num
    idx = (i-1)*cluster_size + (1:cluster_size);  % nodes in cluster
    color = clusters.Color(mod(i-1,cluster_num)+1, :);  % cycle through colors if >13 clusters
    
    % Scatter nodes
    hNodes = scatter(map.lat(idx), map.lon(idx), 60, color, 'filled');
    
    % Scatter centroid
    scatter(clusters.Centroid(i,1), clusters.Centroid(i,2), 80, color, 'filled', 'd');
    
    % Draw cluster circle
    viscircles(clusters.Centroid(i,:), clusters.Radius(i), 'Color', color, 'LineWidth', 1);
end

% Add NodeID label to each node
text(map.lat+0.5, map.lon, cellstr(num2str(map.NodeId)), 'Vert', 'bottom', 'Horiz', 'left', 'FontSize', 9);
% Add centroid label to each centroid
text(clusters.Centroid(:,1), clusters.Centroid(:,2), "\bfC"+clusters.ClusterID+"\rm", 'Vert', 'bottom', 'Horiz', 'left', 'FontSize', 12);


xlabel('X [m]')
ylabel('Y [m]')
title('Local Node Map')

% Override the ButtonDownFcn macro of this figure
ax = gca;
ax.PickableParts = 'visible';
ax.HitTest = 'on';
ax.ButtonDownFcn = @(~,~) delete(findobj(gca,'Tag','NodeTooltip'));

% Add x-axis padding
xl = xlim;
padding = 0.05 * (xl(2) - xl(1));  % 5% of range
xlim([xl(1)-padding, xl(2)+padding]);

% Enable showing node data on click + clear datatips when clicking away
% Set custom ButtonDownFcn on scatter points
hNodes.ButtonDownFcn = @(src, event) showDatatip(src, event, map);


%% Local function to show tooltip on node click
function showDatatip(src, event, map)
    % Delete previous tooltips
    existing = findobj(gca,'Tag','NodeTooltip');
    delete(existing)

    % Find clicked point
    pos = event.IntersectionPoint;  % [x y z]
    xClick = pos(1); 
    yClick = pos(2);

    % Find closest node
    [~, idx] = min(hypot(map.lat - xClick, map.lon - yClick));

    % Create a text object near the node
    txtStr = {
        ['Node ID: ', num2str(map.NodeId(idx))]
        ['Addr: ', char(map.evb1000(idx))]
        ['X: ', num2str(map.lat(idx))]
        ['Y: ', num2str(map.lon(idx))]
    };
    text(map.lat(idx)+1., map.lon(idx)+1., txtStr, ...
        'FontSize', 9, 'BackgroundColor', 'w', 'EdgeColor', 'k', ...
        'Tag', 'NodeTooltip', 'Margin', 3);
end


%% LOOK AT REAL RANGE OF NODES TO DEFINE WORKING AREA 
%% WE LACK A REFERENCE MEASUREMENT SYSTEM to track movement
%% MEASUREMENT VARIANCE FOR EACH NODE/CLUSTER? do field tests
%% IS 1 KALMAN FILTER PER SIMULATION/PER CLUSTER ENOUGH?