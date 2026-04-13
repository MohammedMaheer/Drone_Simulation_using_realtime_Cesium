function drone_sim_launcher()
% DRONE_SIM_LAUNCHER  Pre-flight configuration UI for the drone simulator.
%
%   drone_sim_launcher()
%
%   Opens a configuration window where you can:
%     - Select a drone preset or customize parameters
%     - Choose a flight location
%     - Set flight mode, wind, and environment options
%     - Launch the simulator with a single click

    %% ============================================================
    %% PRESETS DATA
    %% ============================================================
    drone_presets = {
        'standard_quad', 'Standard Quad (450mm)'
        'mini_quad',     'Mini Racing Quad (250mm)'
        'heavy_hex',     'Heavy Hexacopter (680mm)'
        'octo_lift',     'Octo Lifter (1000mm)'
        'micro_tri',     'Micro Tricopter (180mm)'
        'custom',        'Custom...'
    };

    location_presets = {
        'Midtown NYC',    40.748817,  -73.985428, 15,  17
        'San Francisco',  37.774929, -122.419418, 30,  17
        'London',         51.507351,   -0.127758, 22,  17
        'Tokyo',          35.689487,  139.691711, 40,  17
        'Dubai',          25.204849,   55.270782,  6,  17
        'Bengaluru',      12.971599,   77.594566, 920, 17
    };

    battery_types = {'2S', '3S', '4S', '5S', '6S'};
    frame_types   = {'quad_x', 'quad_+', 'tri', 'hex_flat', 'hex_y', 'octo_flat', 'octo_x'};

    %% ============================================================
    %% CREATE FIGURE
    %% ============================================================
    fig_w = 720; fig_h = 620;
    scr = get(0, 'ScreenSize');
    fig_x = round((scr(3) - fig_w) / 2);
    fig_y = round((scr(4) - fig_h) / 2);

    fig = figure('Name', 'Drone Simulator — Pre-Flight Setup', ...
        'NumberTitle', 'off', 'MenuBar', 'none', 'ToolBar', 'none', ...
        'Resize', 'off', 'Color', [0.12 0.12 0.14], ...
        'Position', [fig_x fig_y fig_w fig_h], ...
        'CloseRequestFcn', @on_close);

    % Colors
    bg    = [0.12 0.12 0.14];
    panel = [0.16 0.16 0.19];
    fg    = [0.92 0.92 0.92];
    accent = [0.18 0.65 0.95];
    green  = [0.15 0.75 0.35];
    edit_bg = [0.22 0.22 0.26];
    dim    = [0.55 0.55 0.60];

    %% ============================================================
    %% TITLE BAR
    %% ============================================================
    uicontrol(fig, 'Style', 'text', 'String', 'PRE-FLIGHT SETUP', ...
        'Position', [0 fig_h-45 fig_w 35], ...
        'FontSize', 16, 'FontWeight', 'bold', 'FontName', 'Consolas', ...
        'ForegroundColor', accent, 'BackgroundColor', bg, ...
        'HorizontalAlignment', 'center');

    %% ============================================================
    %% LEFT PANEL — DRONE CONFIGURATION
    %% ============================================================
    lp_x = 15; lp_y = 80; lp_w = 340; lp_h = 480;
    uipanel(fig, 'Title', '  DRONE  ', 'FontSize', 10, 'FontWeight', 'bold', ...
        'FontName', 'Consolas', ...
        'ForegroundColor', accent, 'BackgroundColor', panel, ...
        'HighlightColor', accent, 'ShadowColor', panel, ...
        'Position', [lp_x lp_y lp_w lp_h]);

    % Preset selector
    cy = lp_y + lp_h - 55;
    make_label(lp_x+15, cy, 90, 'Preset:');
    h_preset = uicontrol(fig, 'Style', 'popupmenu', ...
        'String', drone_presets(:,2), 'Value', 1, ...
        'Position', [lp_x+110 cy 220 24], ...
        'FontSize', 9, 'FontName', 'Consolas', ...
        'BackgroundColor', edit_bg, 'ForegroundColor', fg, ...
        'Callback', @on_preset_change);

    % Custom parameters (initially disabled)
    cy = cy - 35;
    make_label(lp_x+15, cy, 90, 'Frame:');
    h_frame = uicontrol(fig, 'Style', 'popupmenu', ...
        'String', frame_types, 'Value', 1, ...
        'Position', [lp_x+110 cy 220 24], ...
        'FontSize', 9, 'FontName', 'Consolas', ...
        'BackgroundColor', edit_bg, 'ForegroundColor', fg, ...
        'Enable', 'off');

    cy = cy - 30;
    [h_mass, ~]     = make_param_row(lp_x, cy, 'Mass (kg):', '1.50');
    cy = cy - 30;
    [h_arm, ~]      = make_param_row(lp_x, cy, 'Arm (m):', '0.225');
    cy = cy - 30;
    [h_prop, ~]     = make_param_row(lp_x, cy, 'Prop (in):', '10.0');
    cy = cy - 30;
    [h_pitch, ~]    = make_param_row(lp_x, cy, 'Pitch (in):', '4.5');
    cy = cy - 30;
    [h_kv, ~]       = make_param_row(lp_x, cy, 'Motor Kv:', '920');
    cy = cy - 30;
    make_label(lp_x+15, cy, 90, 'Battery:');
    h_batt = uicontrol(fig, 'Style', 'popupmenu', ...
        'String', battery_types, 'Value', 3, ...
        'Position', [lp_x+110 cy 220 24], ...
        'FontSize', 9, 'FontName', 'Consolas', ...
        'BackgroundColor', edit_bg, 'ForegroundColor', fg, ...
        'Enable', 'off');
    cy = cy - 30;
    [h_cap, ~]      = make_param_row(lp_x, cy, 'Capacity (Ah):', '5.0');
    cy = cy - 30;
    [h_payload, ~]  = make_param_row(lp_x, cy, 'Payload (kg):', '0.0');

    custom_edits = [h_frame; h_mass; h_arm; h_prop; h_pitch; h_kv; h_batt; h_cap; h_payload];

    % Drone info summary
    cy = cy - 40;
    h_summary = uicontrol(fig, 'Style', 'text', ...
        'String', '', 'Max', 5, ...
        'Position', [lp_x+15 cy 310 35], ...
        'FontSize', 8, 'FontName', 'Consolas', ...
        'ForegroundColor', dim, 'BackgroundColor', panel, ...
        'HorizontalAlignment', 'left');

    %% ============================================================
    %% RIGHT PANEL — ENVIRONMENT & FLIGHT
    %% ============================================================
    rp_x = 370; rp_y = 80; rp_w = 335; rp_h = 480;
    uipanel(fig, 'Title', '  ENVIRONMENT  ', 'FontSize', 10, 'FontWeight', 'bold', ...
        'FontName', 'Consolas', ...
        'ForegroundColor', accent, 'BackgroundColor', panel, ...
        'HighlightColor', accent, 'ShadowColor', panel, ...
        'Position', [rp_x rp_y rp_w rp_h]);

    % Location
    cy = rp_y + rp_h - 55;
    make_label(rp_x+15, cy, 90, 'Location:');
    h_location = uicontrol(fig, 'Style', 'popupmenu', ...
        'String', location_presets(:,1), 'Value', 1, ...
        'Position', [rp_x+110 cy 210 24], ...
        'FontSize', 9, 'FontName', 'Consolas', ...
        'BackgroundColor', edit_bg, 'ForegroundColor', fg, ...
        'Callback', @on_location_change);

    % Lat / Lon / Zoom
    cy = cy - 35;
    make_label(rp_x+15, cy, 60, 'Lat:');
    h_lat = uicontrol(fig, 'Style', 'edit', 'String', '40.748817', ...
        'Position', [rp_x+80 cy 100 22], ...
        'FontSize', 9, 'FontName', 'Consolas', ...
        'BackgroundColor', edit_bg, 'ForegroundColor', fg);
    make_label(rp_x+190, cy, 40, 'Lon:');
    h_lon = uicontrol(fig, 'Style', 'edit', 'String', '-73.985428', ...
        'Position', [rp_x+230 cy 90 22], ...
        'FontSize', 9, 'FontName', 'Consolas', ...
        'BackgroundColor', edit_bg, 'ForegroundColor', fg);

    cy = cy - 30;
    make_label(rp_x+15, cy, 60, 'Zoom:');
    h_zoom = uicontrol(fig, 'Style', 'edit', 'String', '17', ...
        'Position', [rp_x+80 cy 50 22], ...
        'FontSize', 9, 'FontName', 'Consolas', ...
        'BackgroundColor', edit_bg, 'ForegroundColor', fg);

    % Separator
    cy = cy - 20;
    uicontrol(fig, 'Style', 'text', 'String', '─── FLIGHT ───', ...
        'Position', [rp_x+15 cy 305 18], ...
        'FontSize', 9, 'FontName', 'Consolas', ...
        'ForegroundColor', dim, 'BackgroundColor', panel, ...
        'HorizontalAlignment', 'center');

    % Flight mode
    cy = cy - 30;
    make_label(rp_x+15, cy, 90, 'Start mode:');
    h_mode = uicontrol(fig, 'Style', 'popupmenu', ...
        'String', {'Auto (stabilized)', 'Manual (acro)'}, 'Value', 1, ...
        'Position', [rp_x+110 cy 210 24], ...
        'FontSize', 9, 'FontName', 'Consolas', ...
        'BackgroundColor', edit_bg, 'ForegroundColor', fg);

    % Self-level toggle (for manual mode)
    cy = cy - 28;
    h_selflevel = uicontrol(fig, 'Style', 'checkbox', 'Value', 1, ...
        'String', '  Self-level in manual mode', ...
        'Position', [rp_x+15 cy 250 22], ...
        'FontSize', 9, 'FontName', 'Consolas', ...
        'ForegroundColor', fg, 'BackgroundColor', panel);

    % Expo slider
    cy = cy - 30;
    make_label(rp_x+15, cy, 90, 'Stick expo:');
    h_expo = uicontrol(fig, 'Style', 'slider', ...
        'Min', 0, 'Max', 0.8, 'Value', 0.35, ...
        'Position', [rp_x+110 cy 150 20], ...
        'BackgroundColor', panel);
    h_expo_val = uicontrol(fig, 'Style', 'text', 'String', '0.35', ...
        'Position', [rp_x+265 cy 50 18], ...
        'FontSize', 9, 'FontName', 'Consolas', ...
        'ForegroundColor', fg, 'BackgroundColor', panel);
    addlistener(h_expo, 'Value', 'PostSet', @(~,~) set(h_expo_val, ...
        'String', sprintf('%.2f', get(h_expo, 'Value'))));

    % Separator
    cy = cy - 20;
    uicontrol(fig, 'Style', 'text', 'String', '─── OPTIONS ───', ...
        'Position', [rp_x+15 cy 305 18], ...
        'FontSize', 9, 'FontName', 'Consolas', ...
        'ForegroundColor', dim, 'BackgroundColor', panel, ...
        'HorizontalAlignment', 'center');

    % Toggle options
    cy = cy - 28;
    h_wind = uicontrol(fig, 'Style', 'checkbox', 'Value', 1, ...
        'String', '  Enable wind & turbulence', ...
        'Position', [rp_x+15 cy 250 22], ...
        'FontSize', 9, 'FontName', 'Consolas', ...
        'ForegroundColor', fg, 'BackgroundColor', panel);
    cy = cy - 25;
    h_city = uicontrol(fig, 'Style', 'checkbox', 'Value', 1, ...
        'String', '  3D city buildings', ...
        'Position', [rp_x+15 cy 250 22], ...
        'FontSize', 9, 'FontName', 'Consolas', ...
        'ForegroundColor', fg, 'BackgroundColor', panel);
    cy = cy - 25;
    h_traffic = uicontrol(fig, 'Style', 'checkbox', 'Value', 1, ...
        'String', '  Traffic simulation', ...
        'Position', [rp_x+15 cy 250 22], ...
        'FontSize', 9, 'FontName', 'Consolas', ...
        'ForegroundColor', fg, 'BackgroundColor', panel);
    cy = cy - 25;
    h_online = uicontrol(fig, 'Style', 'checkbox', 'Value', 1, ...
        'String', '  Online map tiles (OSM)', ...
        'Position', [rp_x+15 cy 250 22], ...
        'FontSize', 9, 'FontName', 'Consolas', ...
        'ForegroundColor', fg, 'BackgroundColor', panel);
    cy = cy - 25;
    h_battery = uicontrol(fig, 'Style', 'checkbox', 'Value', 1, ...
        'String', '  Precise battery model', ...
        'Position', [rp_x+15 cy 250 22], ...
        'FontSize', 9, 'FontName', 'Consolas', ...
        'ForegroundColor', fg, 'BackgroundColor', panel);

    %% ============================================================
    %% FLY BUTTON
    %% ============================================================
    uicontrol(fig, 'Style', 'pushbutton', ...
        'String', 'FLY', ...
        'Position', [fig_w/2-120 15 240 50], ...
        'FontSize', 20, 'FontWeight', 'bold', 'FontName', 'Consolas', ...
        'ForegroundColor', [1 1 1], 'BackgroundColor', green, ...
        'Callback', @on_fly);

    % Status text
    h_status = uicontrol(fig, 'Style', 'text', 'String', 'Ready to fly.', ...
        'Position', [15 65 fig_w-30 18], ...
        'FontSize', 8, 'FontName', 'Consolas', ...
        'ForegroundColor', dim, 'BackgroundColor', bg, ...
        'HorizontalAlignment', 'center');

    %% Initialize summary with default preset
    on_preset_change(h_preset, []);

    %% ============================================================
    %% CALLBACKS
    %% ============================================================
    function on_preset_change(src, ~)
        idx = get(src, 'Value');
        is_custom = idx == size(drone_presets, 1);

        if is_custom
            enable_str = 'on';
        else
            enable_str = 'off';
        end

        for k = 1:numel(custom_edits)
            set(custom_edits(k), 'Enable', enable_str);
        end

        % Fill fields from preset defaults
        if ~is_custom
            preset_key = drone_presets{idx, 1};
            try
                tmp_cfg = drone_config(preset_key);
                d = tmp_cfg.drone;
                set(h_mass,    'String', sprintf('%.2f', d.mass));
                set(h_arm,     'String', sprintf('%.3f', d.arm_length));
                set(h_prop,    'String', sprintf('%.1f', d.d_prop / 0.0254));
                set(h_pitch,   'String', sprintf('%.1f', d.d_pitch / 0.0254));
                set(h_kv,      'String', sprintf('%d', d.Kv));
                set(h_cap,     'String', sprintf('%.1f', d.capacity_Ah));
                set(h_payload, 'String', '0.0');

                % Battery type
                batt_idx = find(strcmpi(d.battery_type, battery_types), 1);
                if ~isempty(batt_idx); set(h_batt, 'Value', batt_idx); end

                % Frame type
                fr_idx = find(strcmpi(d.frame_type, frame_types), 1);
                if ~isempty(fr_idx); set(h_frame, 'Value', fr_idx); end

                % Summary
                twr = d.max_thrust_total / (d.mass * d.g);
                hover_pct = 100 / twr;
                set(h_summary, 'String', sprintf( ...
                    '%dM %s  TWR=%.1f  Hover≈%.0f%%  %.0fWh', ...
                    d.num_motors, upper(d.frame_type), twr, hover_pct, d.energy_Wh));
            catch
                set(h_summary, 'String', '');
            end
        else
            set(h_summary, 'String', 'Custom configuration — set parameters above');
        end
    end

    function on_location_change(src, ~)
        idx = get(src, 'Value');
        set(h_lat, 'String', sprintf('%.6f', location_presets{idx, 2}));
        set(h_lon, 'String', sprintf('%.6f', location_presets{idx, 3}));
        set(h_zoom, 'String', sprintf('%d', location_presets{idx, 5}));
    end

    function on_fly(~, ~)
        set(h_status, 'String', 'Building configuration...');
        drawnow;

        try
            cfg = build_config();
            set(h_status, 'String', 'Launching simulator...');
            drawnow;
            delete(fig);
            live_drone_sim(cfg);
        catch err
            set(h_status, 'String', ['Error: ' err.message]);
            set(h_status, 'ForegroundColor', [0.95 0.3 0.3]);
        end
    end

    function on_close(~, ~)
        delete(fig);
    end

    %% ============================================================
    %% BUILD CONFIG STRUCT
    %% ============================================================
    function cfg = build_config()
        preset_idx = get(h_preset, 'Value');
        is_custom = preset_idx == size(drone_presets, 1);

        % Drone config
        if is_custom
            cfg = drone_config('auto', ...
                'NumMotors', frame_to_motors(frame_types{get(h_frame, 'Value')}), ...
                'FrameType', frame_types{get(h_frame, 'Value')}, ...
                'Mass',      str2double(get(h_mass, 'String')), ...
                'ArmLength', str2double(get(h_arm, 'String')), ...
                'PropDiameter', str2double(get(h_prop, 'String')) * 0.0254, ...
                'PropPitch',    str2double(get(h_pitch, 'String')) * 0.0254, ...
                'MotorKv',      str2double(get(h_kv, 'String')), ...
                'BatteryType',  battery_types{get(h_batt, 'Value')}, ...
                'BatteryCapacity', str2double(get(h_cap, 'String')));
            payload = str2double(get(h_payload, 'String'));
            if payload > 0
                cfg.drone.mass = cfg.drone.mass + payload;
                cfg.drone.payload_mass = payload;
            end
        else
            preset_key = drone_presets{preset_idx, 1};
            cfg = drone_config(preset_key);
        end

        % Map / environment overrides
        cfg.map.lat  = str2double(get(h_lat, 'String'));
        cfg.map.lon  = str2double(get(h_lon, 'String'));
        cfg.map.zoom = str2double(get(h_zoom, 'String'));
        cfg.map.enable_city    = get(h_city, 'Value');
        cfg.map.enable_traffic = get(h_traffic, 'Value');
        cfg.map.online         = get(h_online, 'Value');

        % Flight settings
        if get(h_mode, 'Value') == 2
            cfg.flight_mode = 'manual';
        else
            cfg.flight_mode = 'auto';
        end
        cfg.self_level = get(h_selflevel, 'Value');
        cfg.expo       = get(h_expo, 'Value');
        cfg.enable_wind    = get(h_wind, 'Value');
        cfg.precise_battery = get(h_battery, 'Value');
    end

    %% ============================================================
    %% HELPERS
    %% ============================================================
    function h = make_label(x, y, w, txt)
        h = uicontrol(fig, 'Style', 'text', 'String', txt, ...
            'Position', [x y w 20], ...
            'FontSize', 9, 'FontName', 'Consolas', ...
            'ForegroundColor', fg, 'BackgroundColor', panel, ...
            'HorizontalAlignment', 'right');
    end

    function [h_edit, h_lbl] = make_param_row(px, y, label, default_val)
        h_lbl = make_label(px+15, y, 90, label);
        h_edit = uicontrol(fig, 'Style', 'edit', 'String', default_val, ...
            'Position', [px+110 y 220 22], ...
            'FontSize', 9, 'FontName', 'Consolas', ...
            'BackgroundColor', edit_bg, 'ForegroundColor', fg, ...
            'Enable', 'off');
    end

    function n = frame_to_motors(ft)
        switch ft
            case {'tri'};                          n = 3;
            case {'quad_x', 'quad_+'};             n = 4;
            case {'hex_flat', 'hex_y'};            n = 6;
            case {'octo_flat', 'octo_x'};          n = 8;
            otherwise;                             n = 4;
        end
    end
end
