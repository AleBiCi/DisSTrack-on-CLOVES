function room = build_room_constraint(varargin)
%BUILD_ROOM_CONSTRAINT Interactively prepare an occupancy constraint model.
%   ROOM = BUILD_ROOM_CONSTRAINT() loads the DISI map, asks the user to:
%   1) click calibration anchors on the floorplan,
%   2) fit a pixel->metric transform,
%   3) draw the walkable region polygon(s),
%   4) build and save an occupancy grid model.
%
%   The resulting ROOM struct can be saved to MAT and then used at runtime
%   by APPLY_ROOM_CONSTRAINT to validate/correct EKF poses.

    % Quick switches (edit here when launching without name-value args):
    %   quick_zone: "DEPT" or "HALL"
    %   quick_map_image_path_dept / quick_map_image_path_hall:
    %       set both paths once, then the function picks the right one by zone.
    %       Leave empty ("") to use automatic defaults/discovery.
    quick_zone = "HALL"; 
    % USE YOUR PATH
    quick_map_image_path_dept = "C:\Users\frass\OneDrive\Desktop\UNI\Magistrale - Mechatronics Engineering - Unitn\Anno 5\Intelligent Distributed system\Project\references\disi_povo1_map.png";
    quick_map_image_path_hall = "C:\Users\frass\OneDrive\Desktop\UNI\Magistrale - Mechatronics Engineering - Unitn\Anno 5\Intelligent Distributed system\Project\references\p2_ground_floor.png";

    if ~has_name_arg(varargin, 'zone')
        varargin = [{'zone', char(quick_zone)}, varargin];
    end

    requested_zone = normalize_zone(get_name_arg_value(varargin, 'zone', quick_zone));
    switch requested_zone
        case "DEPT"
            quick_map_image_path = quick_map_image_path_dept;
        case "HALL"
            quick_map_image_path = quick_map_image_path_hall;
        otherwise
            quick_map_image_path = "";
    end

    if strlength(strtrim(string(quick_map_image_path))) > 0 && ~has_name_arg(varargin, 'map_image_path')
        varargin = [{'map_image_path', char(quick_map_image_path)}, varargin];
    end

    opts = parse_inputs(varargin{:});
    fprintf('[room] Zone=%s | map=%s | anchors=%s\n', opts.zone, opts.map_image_path, opts.anchor_csv_path);
    anchor_table = load_anchor_map(opts.anchor_csv_path, opts.tracking_node_ids);
    map_image = imread(opts.map_image_path);

    calibration_metric = lookup_anchor_xy(anchor_table, opts.calibration_node_ids, opts.anchor_csv_path);

    fig = figure('Name', 'Room Constraint Builder', 'NumberTitle', 'off');
    imshow(map_image);
    axis image;
    hold on;
    title(sprintf('Click anchors %s in this order', num2str(opts.calibration_node_ids)));

    [u, v] = ginput(numel(opts.calibration_node_ids));
    calibration_pix = [u, v];
    plot(calibration_pix(:,1), calibration_pix(:,2), 'ro', 'MarkerSize', 8, 'LineWidth', 1.5);
    for idx = 1:numel(opts.calibration_node_ids)
        text(calibration_pix(idx,1) + 8, calibration_pix(idx,2), num2str(opts.calibration_node_ids(idx)), ...
            'Color', 'r', 'FontWeight', 'bold');
    end

    tform = fitgeotrans(calibration_pix, calibration_metric, opts.transform_type);

    walkable_polygons_pix = draw_walkable_polygons(map_image);
    walkable_polygons_metric = cell(size(walkable_polygons_pix));
    for poly_idx = 1:numel(walkable_polygons_pix)
        poly = walkable_polygons_pix{poly_idx};
        [wx, wy] = transformPointsForward(tform, poly(:,1), poly(:,2));
        walkable_polygons_metric{poly_idx} = [wx, wy];
    end

    [grid_def, free_mask] = build_grid_from_polygons( ...
        walkable_polygons_metric, anchor_table.xy, opts.cell_size_m, opts.roi_padding_m);

    room = struct();
    room.created_at = char(datetime('now', 'TimeZone', 'local', 'Format', 'yyyy-MM-dd HH:mm:ss Z'));
    room.zone = opts.zone;
    room.map_image_path = opts.map_image_path;
    room.anchor_csv_path = opts.anchor_csv_path;
    room.tracking_node_ids = opts.tracking_node_ids(:).';
    room.transform_type = opts.transform_type;
    room.calibration = struct( ...
        'node_ids', opts.calibration_node_ids(:).', ...
        'pix', calibration_pix, ...
        'metric', calibration_metric);
    room.tform = tform;
    room.walkable_polygons_pix = walkable_polygons_pix;
    room.walkable_polygons_metric = walkable_polygons_metric;
    room.grid = struct( ...
        'cell_size_m', opts.cell_size_m, ...
        'x_edges', grid_def.x_edges, ...
        'y_edges', grid_def.y_edges, ...
        'x_centers', grid_def.x_centers, ...
        'y_centers', grid_def.y_centers, ...
        'free_mask', free_mask, ...
        'free_points', grid_def.free_points, ...
        'occupied_points', grid_def.occupied_points, ...
        'x_limits', [grid_def.x_edges(1), grid_def.x_edges(end)], ...
        'y_limits', [grid_def.y_edges(1), grid_def.y_edges(end)]);

    save(opts.output_mat_path, 'room');
    fprintf('[room] Saved room constraint model to %s\n', opts.output_mat_path);

    show_metric_preview(anchor_table, room);

    if ishghandle(fig)
        figure(fig);
        title('Room constraint model created');
    end
end

function tf = has_name_arg(args, name)
    tf = false;
    if isempty(args)
        return;
    end

    n = numel(args);
    upper_bound = n;
    if mod(n, 2) == 1
        upper_bound = n - 1;
    end

    for idx = 1:2:upper_bound
        key = args{idx};
        if (ischar(key) || (isstring(key) && isscalar(key))) && strcmpi(char(string(key)), name)
            tf = true;
            return;
        end
    end
end

function value = get_name_arg_value(args, name, fallback)
    value = fallback;
    if isempty(args)
        return;
    end

    n = numel(args);
    upper_bound = n;
    if mod(n, 2) == 1
        upper_bound = n - 1;
    end

    for idx = 1:2:upper_bound
        key = args{idx};
        if (ischar(key) || (isstring(key) && isscalar(key))) && strcmpi(char(string(key)), name)
            value = args{idx + 1};
            return;
        end
    end
end

function opts = parse_inputs(varargin)
    here = fileparts(mfilename('fullpath'));
    project_root = fileparts(here);

    parser = inputParser;
    parser.addParameter('zone', 'DEPT', @ischarlike);
    parser.addParameter('map_image_path', '', @ischarlike);
    parser.addParameter('anchor_csv_path', '', @ischarlike);
    parser.addParameter('output_mat_path', '', @ischarlike);
    parser.addParameter('tracking_node_ids', [], @isnumeric);
    parser.addParameter('calibration_node_ids', [], @isnumeric);
    parser.addParameter('transform_type', 'affine', @(s) any(strcmpi(string(s), ["affine", "projective"])));
    parser.addParameter('cell_size_m', 0.25, @(x) isnumeric(x) && isscalar(x) && x > 0);
    parser.addParameter('roi_padding_m', 1.5, @(x) isnumeric(x) && isscalar(x) && x >= 0);
    parser.parse(varargin{:});
    opts = parser.Results;

    zone_key = normalize_zone(opts.zone);
    defaults = zone_defaults(zone_key, here, project_root);

    if strlength(strtrim(string(opts.map_image_path))) == 0
        opts.map_image_path = defaults.map_image_path;
    end
    if strlength(strtrim(string(opts.anchor_csv_path))) == 0
        opts.anchor_csv_path = defaults.anchor_csv_path;
    end
    if strlength(strtrim(string(opts.output_mat_path))) == 0
        opts.output_mat_path = defaults.output_mat_path;
    end
    if isempty(opts.tracking_node_ids)
        opts.tracking_node_ids = defaults.tracking_node_ids;
    end
    if isempty(opts.calibration_node_ids)
        opts.calibration_node_ids = defaults.calibration_node_ids;
    end

    opts.zone = zone_key;
    opts.map_image_path = char(opts.map_image_path);
    opts.anchor_csv_path = char(opts.anchor_csv_path);
    opts.output_mat_path = char(opts.output_mat_path);

    if ~isfile(opts.anchor_csv_path)
        error('[room] Anchor CSV not found for zone %s: %s', opts.zone, opts.anchor_csv_path);
    end
    if ~isfile(opts.map_image_path)
        error(['[room] Map image not found for zone %s: %s\n', ...
               'Pass map_image_path explicitly, e.g. build_room_constraint(''zone'',''%s'',''map_image_path'',''/abs/path/image.png'').'], ...
               opts.zone, opts.map_image_path, opts.zone);
    end
end

function tf = ischarlike(value)
    tf = ischar(value) || (isstring(value) && isscalar(value));
end

function anchor_table = load_anchor_map(csv_path, tracking_node_ids)
    raw = importfile(csv_path);
    raw = raw(ismember(raw.NodeId, tracking_node_ids), :);

    node_coords = split(raw.Coordinates, ",");
    x = str2double(regexprep(node_coords(:,1), '[\[\]]', ''));
    y = str2double(regexprep(node_coords(:,2), '[\[\]]', ''));

    anchor_table = raw;
    anchor_table.x = x;
    anchor_table.y = y;
    anchor_table.xy = [x, y];
end

function metric_xy = lookup_anchor_xy(anchor_table, node_ids, csv_path)
    metric_xy = zeros(numel(node_ids), 2);
    for idx = 1:numel(node_ids)
        match = find(anchor_table.NodeId == node_ids(idx), 1);
        if isempty(match)
            [~, csv_name, csv_ext] = fileparts(csv_path);
            error('Anchor node %d not found in %s%s.', node_ids(idx), csv_name, csv_ext);
        end
        metric_xy(idx, :) = anchor_table.xy(match, :);
    end
end

function zone_key = normalize_zone(zone_value)
    zone_key = upper(strtrim(string(zone_value)));
    if zone_key == "HALL-A" || zone_key == "HALL_A"
        zone_key = "HALL";
    end
    if zone_key ~= "DEPT" && zone_key ~= "HALL"
        error('[room] Unsupported zone "%s". Use "DEPT" or "HALL".', zone_value);
    end
end

function defaults = zone_defaults(zone_key, here, project_root)
    references_dir = fullfile(project_root, 'references');

    switch zone_key
        case "DEPT"
            defaults = struct( ...
                'map_image_path', fullfile(references_dir, 'disi_povo1_map.png'), ...
                'anchor_csv_path', fullfile(here, 'DEPT_evb1000_map.csv'), ...
                'output_mat_path', fullfile(here, 'room_constraint_model_dept.mat'), ...
                'tracking_node_ids', [108, 113:119, 121:154], ...
                'calibration_node_ids', [108, 122, 137, 148]);
        case "HALL"
            defaults = struct( ...
                'map_image_path', discover_hall_map_image(references_dir), ...
                'anchor_csv_path', fullfile(here, 'HALL-A_evb1000_map.csv'), ...
                'output_mat_path', fullfile(here, 'room_constraint_model_hall.mat'), ...
                'tracking_node_ids', [50:58, 61:65, 70:77], ...
                'calibration_node_ids', [54, 56, 50, 65]);
        otherwise
            error('[room] Unsupported zone "%s".', zone_key);
    end
end

function map_image_path = discover_hall_map_image(references_dir)
    fixed_candidates = { ...
        fullfile(references_dir, 'hall_map.png'), ...
        fullfile(references_dir, 'HALL-A_map.png'), ...
        fullfile(references_dir, 'hall_a_map.png'), ...
        fullfile(references_dir, 'hall_map.jpg'), ...
        fullfile(references_dir, 'hall_map.jpeg')};

    for idx = 1:numel(fixed_candidates)
        if isfile(fixed_candidates{idx})
            map_image_path = fixed_candidates{idx};
            return;
        end
    end

    discovered = [ ...
        dir(fullfile(references_dir, '*hall*.png')); ...
        dir(fullfile(references_dir, '*hall*.jpg')); ...
        dir(fullfile(references_dir, '*hall*.jpeg'))];
    if ~isempty(discovered)
        map_image_path = fullfile(discovered(1).folder, discovered(1).name);
        return;
    end

    map_image_path = fullfile(references_dir, 'hall_map.png');
end

function polygons = draw_walkable_polygons(map_image)
    polygons = {};
    fig = figure('Name', 'Walkable Region', 'NumberTitle', 'off');
    imshow(map_image);
    axis image;
    hold on;
    title({'Draw the walkable region polygon.', ...
           'Double-click to close each polygon. Press Cancel when done.'});

    keep_drawing = true;
    while keep_drawing
        h = drawpolygon('LineWidth', 1.5, 'Color', 'g');
        wait(h);
        if isempty(h) || ~isvalid(h)
            break;
        end
        poly = h.Position;
        if size(poly, 1) < 3
            delete(h);
            break;
        end
        polygons{end+1} = poly; %#ok<AGROW>

        choice = questdlg('Add another walkable polygon?', 'Walkable region', 'Yes', 'No', 'No');
        keep_drawing = strcmp(choice, 'Yes');
    end

    if isempty(polygons)
        error('No walkable polygon was provided. Room model creation aborted.');
    end

    if ishghandle(fig)
        close(fig);
    end
end

function [grid_def, free_mask] = build_grid_from_polygons(polygons_metric, anchor_xy, cell_size_m, roi_padding_m)
    poly_points = vertcat(polygons_metric{:});

    x_min = floor((min([poly_points(:,1); anchor_xy(:,1)]) - roi_padding_m) / cell_size_m) * cell_size_m;
    x_max = ceil((max([poly_points(:,1); anchor_xy(:,1)]) + roi_padding_m) / cell_size_m) * cell_size_m;
    y_min = floor((min([poly_points(:,2); anchor_xy(:,2)]) - roi_padding_m) / cell_size_m) * cell_size_m;
    y_max = ceil((max([poly_points(:,2); anchor_xy(:,2)]) + roi_padding_m) / cell_size_m) * cell_size_m;

    x_edges = x_min:cell_size_m:x_max;
    y_edges = y_min:cell_size_m:y_max;

    if numel(x_edges) < 2 || numel(y_edges) < 2
        error('Invalid occupancy grid limits. Check drawn polygons and padding.');
    end

    x_centers = x_edges(1:end-1) + cell_size_m / 2;
    y_centers = y_edges(1:end-1) + cell_size_m / 2;
    [Xc, Yc] = meshgrid(x_centers, y_centers);

    free_mask = false(size(Xc));
    for poly_idx = 1:numel(polygons_metric)
        poly = polygons_metric{poly_idx};
        free_mask = free_mask | inpolygon(Xc, Yc, poly(:,1), poly(:,2));
    end

    free_points = [Xc(free_mask), Yc(free_mask)];
    occupied_points = [Xc(~free_mask), Yc(~free_mask)];
    if isempty(free_points)
        error('No free cells were generated. Check polygon placement or cell size.');
    end

    grid_def = struct( ...
        'x_edges', x_edges, ...
        'y_edges', y_edges, ...
        'x_centers', x_centers, ...
        'y_centers', y_centers, ...
        'free_points', free_points, ...
        'occupied_points', occupied_points);
end

function show_metric_preview(anchor_table, room)
    figure('Name', 'Room Constraint Preview', 'NumberTitle', 'off');
    hold on;
    grid on;
    axis equal;

    free_points = room.grid.free_points;
    plot(free_points(:,1), free_points(:,2), '.', 'Color', [0.85, 0.9, 0.85], 'MarkerSize', 8);
    plot(anchor_table.x, anchor_table.y, 'b^', 'MarkerSize', 8, 'LineWidth', 1.2);

    for poly_idx = 1:numel(room.walkable_polygons_metric)
        poly = room.walkable_polygons_metric{poly_idx};
        plot([poly(:,1); poly(1,1)], [poly(:,2); poly(1,2)], 'g-', 'LineWidth', 1.5);
    end

    text(anchor_table.x + 0.2, anchor_table.y + 0.2, string(anchor_table.NodeId), 'Color', 'b');
    xlabel('X [m]');
    ylabel('Y [m]');
    title('Metric occupancy preview');
end
