function [xy_out, info] = apply_distance_penalty(room, xy_in, varargin)
%APPLY_DISTANCE_PENALTY Softly push a valid pose away from nearby walls.
%   [XY_OUT, INFO] = APPLY_DISTANCE_PENALTY(ROOM, XY_IN) keeps XY_IN fixed
%   when it is safely inside the walkable region. If XY_IN is free but
%   closer than SAFE_DISTANCE_M to occupied space, the function computes a
%   bounded correction away from the nearest occupied cell and then clamps
%   the candidate pose back into free space if needed.
%
%   Name-value parameters:
%       'safe_distance_m'  default 0.75
%       'gain'             default 0.60
%       'max_push_m'       default 0.25
%       'label'            default 'pose'

    parser = inputParser;
    parser.addParameter('safe_distance_m', 0.75, @(x) isnumeric(x) && isscalar(x) && x > 0);
    parser.addParameter('gain', 0.60, @(x) isnumeric(x) && isscalar(x) && x >= 0);
    parser.addParameter('max_push_m', 0.25, @(x) isnumeric(x) && isscalar(x) && x >= 0);
    parser.addParameter('label', 'pose', @(s) ischar(s) || (isstring(s) && isscalar(s)));
    parser.parse(varargin{:});
    opts = parser.Results;

    xy = reshape(double(xy_in), 1, 2);
    room_zone = extract_room_zone_local(room);
    [is_free, grid_idx] = is_pose_free_local(room, xy);

    nearest_occ_xy = [NaN, NaN];
    wall_distance = NaN;
    push_m = 0.0;
    applied = false;
    action = "keep";
    xy_out = xy;

    if ~is_free
        action = "skip_outside";
    else
        [nearest_occ_xy, wall_distance] = nearest_occupied_point(room, xy);
        if ~isnan(wall_distance) && wall_distance < opts.safe_distance_m
            dir_vec = xy - nearest_occ_xy;
            dir_norm = norm(dir_vec);

            if dir_norm > 1e-9
                push_m = min(opts.max_push_m, opts.gain * (opts.safe_distance_m - wall_distance));
                candidate_xy = xy + (push_m / dir_norm) * dir_vec;

                [candidate_xy, room_info] = apply_room_constraint( ...
                    room, candidate_xy, ...
                    'projection_threshold_m', opts.safe_distance_m, ...
                    'fallback_pose', xy, ...
                    'label', [char(opts.label) '_distance_penalty']);

                xy_out = candidate_xy;
                applied = norm(xy_out - xy) > 1e-6;
                if applied
                    action = "push";
                elseif strcmp(room_info.action, 'project')
                    action = "project";
                end
            end
        end
    end

    info = struct( ...
        'zone', room_zone, ...
        'label', char(opts.label), ...
        'action', char(action), ...
        'applied', applied, ...
        'was_free', is_free, ...
        'grid_idx', grid_idx, ...
        'safe_distance_m', opts.safe_distance_m, ...
        'wall_distance_m', wall_distance, ...
        'push_m', push_m, ...
        'nearest_occupied_xy', nearest_occ_xy, ...
        'input_xy', xy, ...
        'output_xy', xy_out);
end

function zone = extract_room_zone_local(room)
    if isstruct(room) && isfield(room, 'zone') && ~isempty(room.zone)
        zone = char(string(room.zone));
    else
        zone = 'UNKNOWN';
    end
end

function [is_free, idx] = is_pose_free_local(room, xy)
    idx = pose_to_index_local(room, xy);
    if any(isnan(idx))
        is_free = false;
        return;
    end
    is_free = room.grid.free_mask(idx(2), idx(1));
end

function idx = pose_to_index_local(room, xy)
    x_edges = room.grid.x_edges;
    y_edges = room.grid.y_edges;

    col = discretize(xy(1), x_edges);
    row = discretize(xy(2), y_edges);

    if isempty(col) || isempty(row) || isnan(col) || isnan(row)
        idx = [NaN, NaN];
    else
        idx = [col, row];
    end
end

function [xy_occ, distance_to_occ] = nearest_occupied_point(room, xy)
    occupied_points = get_occupied_points(room);
    if isempty(occupied_points)
        xy_occ = [NaN, NaN];
        distance_to_occ = NaN;
        return;
    end

    diff_xy = occupied_points - xy;
    sq_dist = sum(diff_xy.^2, 2);
    [distance_sq_min, idx_min] = min(sq_dist);
    xy_occ = occupied_points(idx_min, :);
    distance_to_occ = sqrt(distance_sq_min);
end

function occupied_points = get_occupied_points(room)
    if isfield(room.grid, 'occupied_points') && ~isempty(room.grid.occupied_points)
        occupied_points = room.grid.occupied_points;
        return;
    end

    [Xc, Yc] = meshgrid(room.grid.x_centers, room.grid.y_centers);
    occupied_mask = ~room.grid.free_mask;
    occupied_points = [Xc(occupied_mask), Yc(occupied_mask)];
end
