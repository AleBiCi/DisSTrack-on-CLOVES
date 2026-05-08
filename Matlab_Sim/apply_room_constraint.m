function [xy_out, info] = apply_room_constraint(room, xy_in, varargin)
%APPLY_ROOM_CONSTRAINT Validate or correct a 2D pose against a room model.
%   [XY_OUT, INFO] = APPLY_ROOM_CONSTRAINT(ROOM, XY_IN) checks whether
%   XY_IN = [x y] lies inside the walkable occupancy region.
%
%   If XY_IN is free, it is kept.
%   If XY_IN is outside but close to the free region, it is projected to
%   the nearest free grid point.
%   If XY_IN is too far from free space, the pose is rejected and the
%   optional fallback pose is returned instead.
%
%   Name-value parameters:
%       'projection_threshold_m'  default 0.40
%       'fallback_pose'           default []
%       'label'                   default 'pose'

    parser = inputParser;
    parser.addParameter('projection_threshold_m', 0.40, @(x) isnumeric(x) && isscalar(x) && x > 0);
    parser.addParameter('fallback_pose', [], @(x) isnumeric(x) && (isempty(x) || numel(x) == 2));
    parser.addParameter('label', 'pose', @(s) ischar(s) || (isstring(s) && isscalar(s)));
    parser.parse(varargin{:});
    opts = parser.Results;

    xy = reshape(double(xy_in), 1, 2);
    room_zone = extract_room_zone(room);
    fallback_pose = reshape_fallback(opts.fallback_pose);

    [is_free, grid_idx] = is_pose_free(room, xy);
    nearest_xy = [NaN, NaN];
    distance_to_free = 0.0;

    if is_free
        xy_out = xy;
        action = "keep";
        accepted = true;
    else
        [nearest_xy, distance_to_free] = nearest_free_point(room, xy);
        if distance_to_free <= opts.projection_threshold_m
            xy_out = nearest_xy;
            action = "project";
            accepted = true;
        elseif ~isempty(fallback_pose)
            xy_out = fallback_pose;
            action = "reject";
            accepted = false;
        else
            xy_out = xy;
            action = "reject";
            accepted = false;
        end
    end

    info = struct( ...
        'zone', room_zone, ...
        'label', char(opts.label), ...
        'accepted', accepted, ...
        'action', char(action), ...
        'was_free', is_free, ...
        'grid_idx', grid_idx, ...
        'distance_to_free_m', distance_to_free, ...
        'nearest_free_xy', nearest_xy, ...
        'input_xy', xy, ...
        'output_xy', xy_out);
end

function zone = extract_room_zone(room)
    if isstruct(room) && isfield(room, 'zone') && ~isempty(room.zone)
        zone = char(string(room.zone));
    else
        zone = 'UNKNOWN';
    end
end

function fallback_pose = reshape_fallback(fallback_pose)
    if isempty(fallback_pose)
        return;
    end
    fallback_pose = reshape(double(fallback_pose), 1, 2);
end

function [is_free, idx] = is_pose_free(room, xy)
    idx = pose_to_index(room, xy);
    if any(isnan(idx))
        is_free = false;
        return;
    end
    is_free = room.grid.free_mask(idx(2), idx(1));
end

function idx = pose_to_index(room, xy)
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

function [xy_free, distance_to_free] = nearest_free_point(room, xy)
    free_points = room.grid.free_points;
    diff_xy = free_points - xy;
    sq_dist = sum(diff_xy.^2, 2);
    [distance_sq_min, idx_min] = min(sq_dist);
    xy_free = free_points(idx_min, :);
    distance_to_free = sqrt(distance_sq_min);
end
