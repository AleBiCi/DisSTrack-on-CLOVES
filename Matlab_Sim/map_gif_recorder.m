function varargout = map_gif_recorder(action, varargin)
%MAP_GIF_RECORDER Record a live MATLAB figure to a temporary GIF.
% Usage:
%   recorder = map_gif_recorder('start', cfg);
%   recorder = map_gif_recorder('capture', recorder, fig_handle);
%   map_gif_recorder('prompt_save', recorder);

    action = lower(strtrim(char(action)));

    switch action
        case 'start'
            if nargin < 2
                error('map_gif_recorder:start', 'Missing GIF recorder config.');
            end
            varargout{1} = start_recorder(varargin{1});

        case 'capture'
            if nargin < 3
                error('map_gif_recorder:capture', 'Missing recorder state or figure handle.');
            end
            varargout{1} = capture_recorder_frame(varargin{1}, varargin{2});

        case 'prompt_save'
            if nargin < 2
                error('map_gif_recorder:prompt_save', 'Missing recorder state.');
            end
            prompt_save_recorder(varargin{1});
            if nargout > 0
                varargout{1} = varargin{1};
            end

        otherwise
            error('map_gif_recorder:unknown_action', 'Unknown GIF recorder action: %s', action);
    end
end

function recorder = start_recorder(cfg)
    recorder = empty_recorder();
    recorder.enabled = logical(field_value(cfg, 'enabled', true));
    if ~recorder.enabled
        return;
    end

    recorder.delay_time_s = double(field_value(cfg, 'delay_time_s', 0.5));
    if ~isfinite(recorder.delay_time_s) || recorder.delay_time_s <= 0
        recorder.delay_time_s = 0.5;
    end

    capture_every_n = double(field_value(cfg, 'capture_every_n', 1));
    if ~isfinite(capture_every_n) || capture_every_n < 1
        capture_every_n = 1;
    end
    recorder.capture_every_n = max(1, round(capture_every_n));
    recorder.loop_count = double(field_value(cfg, 'loop_count', Inf));

    output_basename = char(field_value(cfg, 'output_basename', 'map_recording'));
    output_basename = strtrim(output_basename);
    if isempty(output_basename)
        output_basename = 'map_recording';
    end
    output_dir = char(field_value(cfg, 'output_dir', pwd));
    output_dir = strtrim(output_dir);
    if isempty(output_dir)
        output_dir = pwd;
    end
    safe_basename = regexprep(output_basename, '[^\w.-]', '_');
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    temp_timestamp = datestr(now, 'yyyymmdd_HHMMSS_FFF');

    recorder.default_output_name = [safe_basename '_' timestamp '.gif'];
    recorder.default_output_dir = output_dir;
    recorder.temp_path = fullfile(tempdir, [safe_basename '_' temp_timestamp '_tmp.gif']);

    fprintf('[gif] recording map figure in the background\n');
end

function recorder = capture_recorder_frame(recorder, fig_handle)
    if ~is_valid_recorder(recorder) || ~recorder.enabled || ~isgraphics(fig_handle)
        return;
    end

    recorder.sample_count = recorder.sample_count + 1;
    if mod(recorder.sample_count - 1, recorder.capture_every_n) ~= 0
        return;
    end

    try
        frame = getframe(fig_handle);
        rgb = frame.cdata;
        if isempty(rgb)
            return;
        end

        frame_size = [size(rgb, 1), size(rgb, 2)];
        if isempty(recorder.frame_size)
            recorder.frame_size = frame_size;
        elseif any(frame_size ~= recorder.frame_size)
            recorder.skipped_count = recorder.skipped_count + 1;
            if recorder.skipped_count <= 3
                fprintf('[gif] skipped a frame after the figure size changed; keep the window size fixed while recording\n');
            end
            return;
        end

        [indexed_frame, color_map] = rgb2ind(rgb, 256);
        if recorder.frame_count == 0
            imwrite(indexed_frame, color_map, recorder.temp_path, 'gif', ...
                'LoopCount', recorder.loop_count, 'DelayTime', recorder.delay_time_s);
        else
            imwrite(indexed_frame, color_map, recorder.temp_path, 'gif', ...
                'WriteMode', 'append', 'DelayTime', recorder.delay_time_s);
        end
        recorder.frame_count = recorder.frame_count + 1;
    catch ME
        recorder.error_count = recorder.error_count + 1;
        if recorder.error_count <= 3
            fprintf('[gif] frame capture skipped: %s\n', ME.message);
        end
    end
end

function prompt_save_recorder(recorder)
    if ~is_valid_recorder(recorder) || ~recorder.enabled
        return;
    end

    if recorder.frame_count < 1 || exist(recorder.temp_path, 'file') ~= 2
        fprintf('[gif] no recorded frames available; nothing to save\n');
        cleanup_temp_gif(recorder.temp_path);
        return;
    end

    if recorder.skipped_count > 0
        fprintf('[gif] recorded %d frames; skipped %d frames with changed figure size\n', ...
            recorder.frame_count, recorder.skipped_count);
    else
        fprintf('[gif] recorded %d frames\n', recorder.frame_count);
    end

    reply = input('[gif] Save recorded simulation window as GIF? [y/N]: ', 's');
    if ~is_yes(reply)
        cleanup_temp_gif(recorder.temp_path);
        fprintf('[gif] recording discarded\n');
        return;
    end

    fprintf('[gif] default save folder: %s\n', recorder.default_output_dir);
    output_path = choose_output_path(recorder.default_output_dir, recorder.default_output_name);
    [ok, msg] = copyfile(recorder.temp_path, output_path, 'f');
    if ~ok
        fprintf('[gif] save failed: %s\n', msg);
        fprintf('[gif] temporary recording kept at %s\n', recorder.temp_path);
        return;
    end

    cleanup_temp_gif(recorder.temp_path);
    fprintf('[gif] saved %d frames to %s\n', recorder.frame_count, output_path);
end

function output_path = choose_output_path(default_output_dir, default_output_name)
    while true
        answer = input(sprintf('[gif] GIF filename [%s]: ', default_output_name), 's');
        answer = strtrim(answer);
        if isempty(answer)
            answer = default_output_name;
        end

        if isempty(regexpi(answer, '\.gif$', 'once'))
            answer = [answer '.gif'];
        end

        answer = expand_home_path(answer);
        if ~is_absolute_path(answer)
            answer = fullfile(default_output_dir, answer);
        end

        [output_dir, ~, ~] = fileparts(answer);
        if ~isempty(output_dir) && exist(output_dir, 'dir') ~= 7
            try
                mkdir(output_dir);
            catch ME
                fprintf('[gif] could not create folder %s: %s\n', output_dir, ME.message);
                continue;
            end
        end

        if exist(answer, 'file') == 2
            overwrite = input(sprintf('[gif] %s exists. Overwrite? [y/N]: ', answer), 's');
            if ~is_yes(overwrite)
                fprintf('[gif] choose another filename\n');
                continue;
            end
        end

        output_path = answer;
        return;
    end
end

function recorder = empty_recorder()
    recorder = struct();
    recorder.enabled = false;
    recorder.delay_time_s = 0.5;
    recorder.capture_every_n = 1;
    recorder.loop_count = Inf;
    recorder.sample_count = 0;
    recorder.frame_count = 0;
    recorder.skipped_count = 0;
    recorder.error_count = 0;
    recorder.frame_size = [];
    recorder.temp_path = '';
    recorder.default_output_name = 'map_recording.gif';
    recorder.default_output_dir = pwd;
end

function value = field_value(s, field_name, default_value)
    value = default_value;
    if isstruct(s) && isfield(s, field_name) && ~isempty(s.(field_name))
        value = s.(field_name);
    end
end

function tf = is_valid_recorder(recorder)
    tf = isstruct(recorder) && isfield(recorder, 'enabled') && ...
        isfield(recorder, 'frame_count') && isfield(recorder, 'temp_path');
end

function tf = is_yes(answer)
    answer = lower(strtrim(answer));
    tf = ~isempty(answer) && answer(1) == 'y';
end

function path_text = expand_home_path(path_text)
    if isempty(path_text) || path_text(1) ~= '~'
        return;
    end

    home_dir = getenv('HOME');
    if isempty(home_dir) && ispc
        home_dir = getenv('USERPROFILE');
    end
    if isempty(home_dir)
        return;
    end

    if strcmp(path_text, '~')
        path_text = home_dir;
    elseif length(path_text) >= 2 && (path_text(2) == '/' || path_text(2) == '\')
        path_text = fullfile(home_dir, path_text(3:end));
    end
end

function tf = is_absolute_path(path_text)
    if ispc
        tf = ~isempty(regexp(path_text, '^[A-Za-z]:[\\/]', 'once')) || ...
            strncmp(path_text, '\\', 2) || strncmp(path_text, '//', 2);
    else
        tf = strncmp(path_text, filesep, length(filesep));
    end
end

function cleanup_temp_gif(temp_path)
    if isempty(temp_path) || exist(temp_path, 'file') ~= 2
        return;
    end

    try
        delete(temp_path);
    catch
    end
end
