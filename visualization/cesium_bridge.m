function result = cesium_bridge(action, data)
% CESIUM_BRIDGE  Manages the CesiumJS 3D globe viewer bridge.
%
%   ok = cesium_bridge('start', env)     — Start HTTP server + open browser
%   ok = cesium_bridge('update', state)  — Push drone state (call each frame)
%   ok = cesium_bridge('stop')           — Shut down server and clean up
%   tf = cesium_bridge('is_running')     — Check if bridge is active
%
%   Uses a Python HTTP server (launched as background process) that
%   serves the CesiumJS viewer and a JSON state endpoint.

    persistent bridge

    if isempty(bridge)
        bridge.active     = false;
        bridge.port       = 8765;
        bridge.state_path = '';
        bridge.pid        = 0;
    end

    result = false;

    switch action
        %% ── START ──────────────────────────────────────────────
        case 'start'
            if bridge.active
                web(sprintf('http://localhost:%d', bridge.port), '-browser');
                result = true;
                return;
            end

            env = data;
            bridge.state_path = fullfile(tempdir, 'drone_cesium_state.json');

            % Write initial state
            s.lat = env.map_lat; s.lon = env.map_lon; s.alt = env.map_alt;
            s.roll = 0; s.pitch = 0; s.yaw = 0;
            s.speed = 0; s.vz = 0; s.soc = 1;
            s.mode = 'auto'; s.time = 0; s.location = '';
            write_json(bridge.state_path, s);

            % Locate support files
            my_dir    = fileparts(mfilename('fullpath'));
            html_path = fullfile(my_dir, 'cesium_viewer.html');
            py_path   = fullfile(my_dir, 'cesium_server.py');

            if ~exist(html_path, 'file')
                fprintf('[CESIUM] ERROR: cesium_viewer.html not found.\n');
                return;
            end
            if ~exist(py_path, 'file')
                fprintf('[CESIUM] ERROR: cesium_server.py not found.\n');
                return;
            end

            % Find Python
            python_exe = find_python();
            if isempty(python_exe)
                fprintf('[CESIUM] ERROR: Python not found. Install Python 3.x.\n');
                return;
            end

            % Kill any stale server on our port
            try
                [~, out] = system(sprintf('netstat -ano | findstr ":%d "', bridge.port));
                if ~isempty(out)
                    lines = strsplit(strtrim(out), '\n');
                    for li = 1:numel(lines)
                        tok = strsplit(strtrim(lines{li}));
                        if numel(tok) >= 5
                            old_pid = str2double(tok{end});
                            if ~isnan(old_pid) && old_pid > 0
                                system(sprintf('taskkill /F /PID %d >nul 2>&1', old_pid));
                            end
                        end
                    end
                    pause(0.3);
                end
            catch
            end

            % Launch Python server as detached process
            cmd = sprintf('start "" /B "%s" "%s" --port %d --html "%s" --state "%s"', ...
                python_exe, py_path, bridge.port, html_path, bridge.state_path);
            system(cmd);

            % Wait for server (up to 5 seconds)
            url_check = sprintf('http://localhost:%d/state', bridge.port);
            server_ok = false;
            for wi = 1:50
                pause(0.1);
                try
                    webread(url_check);
                    server_ok = true;
                    break;
                catch
                end
            end

            if ~server_ok
                fprintf('[CESIUM] WARNING: Server did not respond within 5 s.\n');
                fprintf('[CESIUM] Tried: %s\n', cmd);
                return;
            end

            % Get the PID so we can kill it on cleanup
            try
                [~, out] = system(sprintf('netstat -ano | findstr "LISTENING" | findstr ":%d "', bridge.port));
                tok = strsplit(strtrim(out));
                bridge.pid = str2double(tok{end});
            catch
                bridge.pid = 0;
            end

            % Open browser
            web(sprintf('http://localhost:%d', bridge.port), '-browser');
            bridge.active = true;
            result = true;
            fprintf('[CESIUM] 3D globe viewer ready at http://localhost:%d\n', bridge.port);

        %% ── UPDATE ─────────────────────────────────────────────
        case 'update'
            if bridge.active
                write_json(bridge.state_path, data);
                result = true;
            end

        %% ── STOP ───────────────────────────────────────────────
        case 'stop'
            if bridge.active
                if bridge.pid > 0
                    system(sprintf('taskkill /F /PID %d >nul 2>&1', bridge.pid));
                end
                % Also kill by port as fallback
                try
                    [~, out] = system(sprintf('netstat -ano | findstr "LISTENING" | findstr ":%d "', bridge.port));
                    tok = strsplit(strtrim(out));
                    pid_val = str2double(tok{end});
                    if ~isnan(pid_val) && pid_val > 0
                        system(sprintf('taskkill /F /PID %d >nul 2>&1', pid_val));
                    end
                catch
                end
                bridge.pid = 0;
                bridge.active = false;
                fprintf('[CESIUM] Server stopped.\n');
                result = true;
            end

        %% ── IS_RUNNING ────────────────────────────────────────
        case 'is_running'
            result = bridge.active;
    end
end


%% ─── Write JSON state to disk (atomic: tmp then rename) ──────────
function write_json(path, s)
    json_str = jsonencode(s);
    tmp_path = [path '.tmp'];
    fid = fopen(tmp_path, 'w');
    if fid == -1; return; end
    fwrite(fid, json_str, 'char');
    fclose(fid);
    movefile(tmp_path, path, 'f');
end


%% ─── Find Python executable ─────────────────────────────────────
function exe = find_python()
    exe = '';
    candidates = {'python', 'python3'};
    for ci = 1:numel(candidates)
        [st, ~] = system(sprintf('where %s 2>nul', candidates{ci}));
        if st == 0
            exe = candidates{ci};
            return;
        end
    end
end
