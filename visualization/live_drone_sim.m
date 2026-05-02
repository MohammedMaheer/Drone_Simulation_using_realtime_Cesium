function live_drone_sim(cfg_input)
% LIVE_DRONE_SIM  Interactive 3D drone simulator with configurable parameters.
%
%   live_drone_sim()                           — Default 450mm quad
%   live_drone_sim(cfg)                        — Pass drone_config struct
%   live_drone_sim(drone_config('heavy_hex'))  — Fly a hexacopter
%   live_drone_sim(drone_config('manual', 'NumMotors', 8, ...))  — Custom
%
%   Controls:
%     W / S       — Pitch forward / backward
%     A / D       — Roll left / right
%     Q / E       — Yaw left / right
%     SPACE       — Increase throttle (climb)
%     SHIFT       — Decrease throttle (descend)
%     H           — Toggle auto-hover (intelligent stabilization)
%     M           — Toggle auto/manual flight mode
%     R           — Reset to origin
%     1           — Toggle wind on/off
%     2           — Toggle trail on/off
%     3           — Toggle camera follow / free
%     4           — Toggle precise battery model
%     TAB         — Cycle HUD pages (flight / config / performance)
%     ESC         — Quit
%
%   FLIGHT MODES:
%     MANUAL — Direct rate commands. You control roll/pitch rates and thrust.
%              No stabilization. For experienced pilots.
%     AUTO   — Intelligent hover + stabilization. PID controllers maintain
%              altitude and attitude. Auto-tunes gains based on drone config.
%              Throttle sets target altitude. WASD tilts the drone.

    %% ============================================================
    %% LOAD CONFIGURATION
    %% ============================================================
    if nargin < 1 || isempty(cfg_input)
        cfg = drone_config('standard_quad');
    elseif ischar(cfg_input) || isstring(cfg_input)
        cfg = drone_config(cfg_input);
    else
        cfg = cfg_input;  % Already a config struct
    end

    dp     = cfg.drone;
    cp     = cfg.controller;
    layout = cfg.motor_layout;
    n_mot  = dp.num_motors;

    % Extract optional launcher overrides (from drone_sim_launcher UI)
    launch_flight_mode = 'auto';
    launch_self_level  = true;
    launch_expo        = 0.35;
    launch_wind        = true;
    launch_precise_batt = true;
    if isfield(cfg, 'flight_mode');    launch_flight_mode = cfg.flight_mode; end
    if isfield(cfg, 'self_level');     launch_self_level  = logical(cfg.self_level); end
    if isfield(cfg, 'expo');           launch_expo        = cfg.expo; end
    if isfield(cfg, 'enable_wind');    launch_wind        = logical(cfg.enable_wind); end
    if isfield(cfg, 'precise_battery'); launch_precise_batt = logical(cfg.precise_battery); end

    % Clear persistent vars in any controllers
    clear attitude_controller position_controller altitude_controller flight_controller

    %% ============================================================
    %% SIMULATION STATE
    %% ============================================================
    sim.state        = zeros(12, 1);
    sim.motor_speeds = ones(n_mot, 1) * dp.hover_omega;
    sim.dt_physics   = 0.002;          % 500 Hz physics
    sim.dt_render    = 1/40;           % 40 FPS rendering
    sim.time         = 0;
    sim.running      = true;

    % Battery state
    batt.soc         = 1.0;
    batt.voltage     = dp.V_full;
    batt.current     = 0;
    batt.energy_used = 0;              % [Wh] consumed
    batt.precise     = launch_precise_batt;  % Use precise battery model
    batt.temperature = 25;             % [degC]
    batt.motor_thermal = 25;           % Motor winding temp [degC]
    batt.thermal_warning = false;
    batt.thermal_cutoff  = false;
    batt.motor_eff_derate = 1.0;

    % Wind & Turbulence
    wind_cfg.use_dryden    = launch_wind;  % Use Dryden turbulence (vs simple sinusoidal)
    wind_cfg.steady_wind   = [0; 0; 0]; % Steady-state wind NED [m/s]
    wind_cfg.turb_intensity = 'moderate'; % 'light', 'moderate', 'severe'

    % Auto-mode controller state (integral terms)
    auto.alt_integral  = 0;
    auto.roll_integral  = 0;
    auto.pitch_integral = 0;
    auto.yaw_integral   = 0;
    auto.pos_x_integral = 0;
    auto.pos_y_integral = 0;
    auto.prev_alt_err   = 0;
    auto.prev_roll_err  = 0;
    auto.prev_pitch_err = 0;
    auto.alt_d_filt     = 0;        % Low-pass filtered altitude derivative
    auto.vel_damp_filt  = [0;0];    % Filtered velocity for damping

    % Control inputs (from keyboard)
    ctrl.throttle_input = 0;
    ctrl.roll_input     = 0;
    ctrl.pitch_input    = 0;
    ctrl.yaw_input      = 0;
    ctrl.flight_mode    = launch_flight_mode;  % 'auto' or 'manual'
    ctrl.target_alt     = 0;
    ctrl.target_yaw     = 0;

    % Control scaling (derived from config)
    ctrl.max_tilt       = deg2rad(25);      % Realistic tilt limit
    ctrl.max_yaw_rate   = deg2rad(180);     % deg/s yaw authority
    ctrl.throttle_sens  = dp.max_thrust_total * 0.35;
    ctrl.max_speed      = 15.0;             % m/s horizontal speed cap
    ctrl.max_climb_rate = 5.0;              % m/s vertical speed cap
    ctrl.expo           = launch_expo;       % Expo curve for sticks (0=linear,1=cubic)
    ctrl.self_level     = launch_self_level; % Self-level in manual mode

    % Manual mode rate & attitude PID state
    manual_ctrl.prev_roll_err  = 0;
    manual_ctrl.prev_pitch_err = 0;
    manual_ctrl.prev_yaw_err   = 0;
    manual_ctrl.roll_integral  = 0;
    manual_ctrl.pitch_integral = 0;
    manual_ctrl.throttle_hold  = 0;         % Remembered throttle level

    % Key state
    keys = struct('w',false,'s',false,'a',false,'d',false,...
                  'q',false,'e',false,'space',false,'shift',false);

    % Visual options
    viz.show_trail    = true;
    viz.camera_follow = true;
    viz.show_wind     = launch_wind;
    viz.max_trail     = 2000;
    viz.trail_idx     = 0;
    viz.trail_x       = NaN(viz.max_trail, 1);
    viz.trail_y       = NaN(viz.max_trail, 1);
    viz.trail_z       = NaN(viz.max_trail, 1);
    viz.wind_vec      = [0; 0; 0];
    viz.hud_page      = 1;    % 1=flight, 2=config, 3=performance

    % Camera system (5 modes)
    viz.camera_mode   = 1;    % 1=Chase, 2=FPV, 3=Orbit, 4=Cinematic, 5=Street
    viz.cam_smooth_pos = [0 0 0]; % Smoothed camera position
    viz.cam_smooth_tgt = [0 0 0]; % Smoothed camera target
    viz.cam_orbit_angle = 0;      % Orbit mode auto-rotation angle
    viz.cam_fov       = 60;       % Field of view (adjustable)
    viz.cam_smooth_yaw = 0;       % Smoothed heading for chase cam
    viz.cam_mode_fresh = true;    % True on first frame after mode switch
    viz.cam_zoom      = 1.0;      % Camera distance multiplier (scroll wheel)

    % Real-world environment settings (online map tiles + city street scene)
    env = default_environment_settings(cfg);

    % Cesium 3D globe viewer (optional photogrammetric mode)
    cesium.enabled = false;

    % Performance tracking (comprehensive)
    perf.max_alt        = 0;
    perf.max_speed      = 0;
    perf.max_climb      = 0;
    perf.max_descent    = 0;
    perf.max_tilt       = 0;
    perf.max_g          = 0;       % Peak g-force
    perf.max_power      = 0;       % Peak power [W]
    perf.distance       = 0;       % 3D distance traveled [m]
    perf.distance_2d    = 0;       % Ground distance traveled [m]
    perf.flight_time    = 0;       % Time airborne [s]
    perf.hover_time     = 0;       % Time nearly stationary in air [s]
    perf.energy_total   = 0;       % Total energy used for distance [Wh]
    perf.prev_pos       = [0;0;0];
    perf.prev_vel       = [0;0;0]; % For acceleration calc
    perf.accel_smooth   = [0;0;0]; % Filtered acceleration [m/s^2]
    perf.vibration      = 0;       % Running vibration index

    % Time-series ring buffer for strip charts (last 15 seconds at render rate)
    ts.buf_len = round(15 / sim.dt_render);  % ~600 samples
    ts.idx     = 0;
    ts.time    = NaN(ts.buf_len, 1);
    ts.alt     = NaN(ts.buf_len, 1);
    ts.alt_tgt = NaN(ts.buf_len, 1);
    ts.speed   = NaN(ts.buf_len, 1);
    ts.vz      = NaN(ts.buf_len, 1);
    ts.roll    = NaN(ts.buf_len, 1);
    ts.pitch   = NaN(ts.buf_len, 1);
    ts.yaw     = NaN(ts.buf_len, 1);
    ts.power   = NaN(ts.buf_len, 1);
    ts.gforce  = NaN(ts.buf_len, 1);
    ts.thr_pct = NaN(ts.buf_len, 1);
    ts.mot_rpm = NaN(ts.buf_len, n_mot);

    % Full flight log for post-flight analysis
    log.max_samples = 50000;
    log.idx         = 0;
    log.time        = zeros(log.max_samples, 1);
    log.pos         = zeros(log.max_samples, 3);
    log.vel         = zeros(log.max_samples, 3);
    log.euler       = zeros(log.max_samples, 3);
    log.omega       = zeros(log.max_samples, 3);
    log.mot_rpm     = zeros(log.max_samples, n_mot);
    log.power       = zeros(log.max_samples, 1);
    log.soc         = zeros(log.max_samples, 1);
    log.gforce      = zeros(log.max_samples, 1);
    log.alt_tgt     = zeros(log.max_samples, 1);
    log.thrust_cmd  = zeros(log.max_samples, 1);

    %% ============================================================
    %% CREATE FIGURE & 3D AXES
    %% ============================================================
    frame_name = sprintf('%s — %dM %s', upper(dp.frame_type), n_mot, ...
        ternary(strcmp(cfg.config_mode,'auto'), 'AUTO-CONFIG', 'MANUAL-CONFIG'));
    screen_sz = get(0, 'ScreenSize');
    fig_w = min(screen_sz(3) - 60, 2560);
    fig_h = min(screen_sz(4) - 80, 1440);
    fig = figure('Name', ['LIVE DRONE SIM — ' frame_name], ...
        'NumberTitle', 'off', ...
        'Position', [30, 40, fig_w, fig_h], ...
        'Color', [0.08 0.08 0.12], ...
        'Renderer', 'opengl', ...
        'GraphicsSmoothing', 'on', ...
        'KeyPressFcn',   @(~,e) key_down(e), ...
        'KeyReleaseFcn',  @(~,e) key_up(e), ...
        'WindowScrollWheelFcn', @(~,e) scroll_zoom(e), ...
        'CloseRequestFcn', @close_sim);

    % --- 3D axes with HDR sky and cinematic lighting ---
    ax3d = axes('Parent', fig, 'Position', [0.02 0.32 0.70 0.65]);
    hold(ax3d, 'on'); grid(ax3d, 'on'); axis(ax3d, 'equal');
    set(ax3d, 'Color', [0.42 0.62 0.88], 'GridAlpha', 0.08, ...
        'GridColor', [0.35 0.35 0.35], 'XColor', [0.5 0.5 0.5], ...
        'YColor', [0.5 0.5 0.5], 'ZColor', [0.5 0.5 0.5], ...
        'Clipping', 'off');
    xlabel(ax3d, 'X [m]'); ylabel(ax3d, 'Y [m]'); zlabel(ax3d, 'Alt [m]');
    title(ax3d, frame_name, 'Color', 'w', 'FontSize', 13, 'FontWeight', 'bold');
    view(ax3d, 135, 30);
    ax3d.XLim = [-5 5]; ax3d.YLim = [-5 5]; ax3d.ZLim = [-0.5 10];

    % HDR multi-light setup: Sun (warm) + Sky fill (cool) + Ground bounce + Rim
    set(ax3d, 'AmbientLightColor', [0.22 0.24 0.32]);
    light(ax3d, 'Position', [1 0.4 -0.8], 'Style', 'infinite', ...
        'Color', [1.0 0.92 0.78]);         % Sun: warm directional
    light(ax3d, 'Position', [-0.3 -0.8 -1], 'Style', 'infinite', ...
        'Color', [0.18 0.25 0.45]);         % Sky fill: cool blue
    light(ax3d, 'Position', [0 0 1], 'Style', 'infinite', ...
        'Color', [0.12 0.15 0.08]);         % Ground bounce: subtle warm
    light(ax3d, 'Position', [-1 0.5 -0.3], 'Style', 'infinite', ...
        'Color', [0.10 0.10 0.15]);         % Rim: subtle backlight
    lighting(ax3d, 'gouraud');

    world = draw_ground(ax3d, dp.arm_length, env);
    draw_sky_dome(ax3d, env);

    % --- 3D HUD overlay elements (attitude indicator, compass, altitude) ---
    hud3d = create_3d_hud(ax3d);

    % --- Create realistic 3D drone geometry ONCE (updated per-frame) ---
    [drone_h, geo] = create_drone_geometry(ax3d, layout, dp);
    prop_angle = 0;

    % --- Overlay effects ---
    h_trail    = plot3(ax3d, NaN, NaN, NaN, 'c-', 'LineWidth', 1.5);
    shadow_theta = linspace(0, 2*pi, 20);
    h_shadow   = fill3(ax3d, cos(shadow_theta), sin(shadow_theta), ...
        zeros(1,20), [0 0 0], 'FaceAlpha', 0.12, 'EdgeColor', 'none');
    h_alt_line = plot3(ax3d, [NaN NaN], [NaN NaN], [NaN NaN], ...
        'k:', 'LineWidth', 0.5, 'Color', [0.4 0.4 0.4 0.4]);
    h_wind     = quiver3(ax3d, 0, 0, 5, 0, 0, 0, 0, 'r', 'LineWidth', 2, ...
        'MaxHeadSize', 2, 'Visible', 'off');

    % --- Prop wash / downwash dust particles ---
    n_dust = 24;
    dust.life  = zeros(n_dust, 1);
    dust.pos   = zeros(n_dust, 3);
    dust.vel   = zeros(n_dust, 3);
    dust.alpha = zeros(n_dust, 1);
    dust.h = gobjects(n_dust, 1);
    for di = 1:n_dust
        dust.h(di) = plot3(ax3d, NaN, NaN, NaN, '.', 'Color', [0.75 0.65 0.45], ...
            'MarkerSize', 3);
    end

    % --- Motion trail (fading afterimage for fast flight) ---
    n_ghost = 6;
    ghost_alpha = linspace(0.12, 0.01, n_ghost);
    h_ghost = gobjects(n_ghost, 1);
    for gi = 1:n_ghost
        h_ghost(gi) = plot3(ax3d, NaN, NaN, NaN, 'o', ...
            'Color', [0.3 0.8 1 ghost_alpha(gi)], 'MarkerSize', max(2, 8-gi));
    end
    ghost_pos = NaN(n_ghost, 3);
    ghost_counter = 0;

    %% --- HUD Panel ---
    hud_ax = axes('Parent', fig, 'Position', [0.74 0.02 0.25 0.96]);
    axis(hud_ax, 'off'); set(hud_ax, 'Color', [0.08 0.08 0.12]);

    h_hud = text(hud_ax, 0.05, 0.97, '', 'FontName', 'Consolas', 'FontSize', 10, ...
        'Color', [0 1 0.4], 'VerticalAlignment', 'top', ...
        'Units', 'normalized', 'Interpreter', 'none');

    h_controls = text(hud_ax, 0.05, 0.25, '', 'FontName', 'Consolas', 'FontSize', 8, ...
        'Color', [0.5 0.6 0.7], 'VerticalAlignment', 'top', ...
        'Units', 'normalized', 'Interpreter', 'none');
    set(h_controls, 'String', sprintf([...
        '--- CONTROLS ---\n', ...
        'W/S   Pitch     Q/E  Yaw\n', ...
        'A/D   Roll      H    Hover\n', ...
        'SPC   Climb     M    Mode\n', ...
        'SHF   Descend   R    Reset\n', ...
        'C CamMode(5)  +/- FOV\n', ...
        'L Next city     T Teleport\n', ...
        'V Cesium 3D globe view\n', ...
        '1 Wind  2 Trail  3 Cam\n', ...
        '4 BattModel  5 Charts\n', ...
        'G GPS readout\n', ...
        'TAB HUD pages  ESC Quit']));

    %% --- Real-time strip chart instruments (6 panels below 3D view) ---
    chart_colors = struct('bg', [0.10 0.10 0.15], 'grid', [0.25 0.25 0.30]);
    strip_w = 0.110;  % Width per chart (6 charts)
    strip_h = 0.28;
    strip_y = 0.02;
    strip_gap = 0.005;

    % Chart 1: Altitude
    ax_alt = axes('Parent', fig, 'Position', [0.02, strip_y, strip_w, strip_h]);
    set(ax_alt, 'Color', chart_colors.bg, 'XColor', [0.5 0.5 0.5], 'YColor', [0.5 0.5 0.5], ...
        'FontSize', 6, 'GridColor', chart_colors.grid, 'GridAlpha', 0.4);
    hold(ax_alt, 'on'); grid(ax_alt, 'on');
    title(ax_alt, 'ALT [m]', 'Color', [0.3 0.9 1], 'FontSize', 7, 'FontWeight', 'bold');
    h_strip_alt     = plot(ax_alt, NaN, NaN, 'c-', 'LineWidth', 1.5);
    h_strip_alt_tgt = plot(ax_alt, NaN, NaN, '--', 'Color', [1 0.5 0.2], 'LineWidth', 1);
    ylabel(ax_alt, 'm', 'Color', [0.6 0.6 0.6]);

    % Chart 2: Speed
    ax_spd = axes('Parent', fig, 'Position', [0.02+(strip_w+strip_gap), strip_y, strip_w, strip_h]);
    set(ax_spd, 'Color', chart_colors.bg, 'XColor', [0.5 0.5 0.5], 'YColor', [0.5 0.5 0.5], ...
        'FontSize', 6, 'GridColor', chart_colors.grid, 'GridAlpha', 0.4);
    hold(ax_spd, 'on'); grid(ax_spd, 'on');
    title(ax_spd, 'SPEED [m/s]', 'Color', [0.3 1 0.5], 'FontSize', 7, 'FontWeight', 'bold');
    h_strip_spd = plot(ax_spd, NaN, NaN, '-', 'Color', [0.3 1 0.5], 'LineWidth', 1.5);
    h_strip_vz  = plot(ax_spd, NaN, NaN, '-', 'Color', [1 0.8 0.3], 'LineWidth', 1);
    ylabel(ax_spd, 'm/s', 'Color', [0.6 0.6 0.6]);

    % Chart 3: Attitude
    ax_att = axes('Parent', fig, 'Position', [0.02+2*(strip_w+strip_gap), strip_y, strip_w, strip_h]);
    set(ax_att, 'Color', chart_colors.bg, 'XColor', [0.5 0.5 0.5], 'YColor', [0.5 0.5 0.5], ...
        'FontSize', 6, 'GridColor', chart_colors.grid, 'GridAlpha', 0.4);
    hold(ax_att, 'on'); grid(ax_att, 'on');
    title(ax_att, 'ATT [deg]', 'Color', [1 0.6 0.3], 'FontSize', 7, 'FontWeight', 'bold');
    h_strip_roll  = plot(ax_att, NaN, NaN, 'r-', 'LineWidth', 1.2);
    h_strip_pitch = plot(ax_att, NaN, NaN, '-', 'Color', [0.2 0.7 1], 'LineWidth', 1.2);
    h_strip_yaw   = plot(ax_att, NaN, NaN, '-', 'Color', [0.6 0.6 0.6], 'LineWidth', 0.8);
    ylabel(ax_att, 'deg', 'Color', [0.6 0.6 0.6]);
    legend(ax_att, {'R','P','Y'}, 'TextColor', [0.7 0.7 0.7], 'Color', 'none', ...
        'EdgeColor', 'none', 'FontSize', 5, 'Location', 'northeast');

    % Chart 4: Power & G-force
    ax_pwr = axes('Parent', fig, 'Position', [0.02+3*(strip_w+strip_gap), strip_y, strip_w, strip_h]);
    set(ax_pwr, 'Color', chart_colors.bg, 'XColor', [0.5 0.5 0.5], 'YColor', [0.5 0.5 0.5], ...
        'FontSize', 6, 'GridColor', chart_colors.grid, 'GridAlpha', 0.4);
    hold(ax_pwr, 'on'); grid(ax_pwr, 'on');
    title(ax_pwr, 'PWR/G', 'Color', [1 0.3 0.4], 'FontSize', 7, 'FontWeight', 'bold');
    yyaxis(ax_pwr, 'left');
    h_strip_pwr = plot(ax_pwr, NaN, NaN, '-', 'Color', [1 0.3 0.4], 'LineWidth', 1.5);
    ylabel(ax_pwr, 'W', 'Color', [1 0.3 0.4]);
    set(ax_pwr, 'YColor', [1 0.3 0.4]);
    yyaxis(ax_pwr, 'right');
    h_strip_g = plot(ax_pwr, NaN, NaN, '-', 'Color', [1 1 0.3], 'LineWidth', 1);
    ylabel(ax_pwr, 'G', 'Color', [1 1 0.3]);
    set(ax_pwr, 'YColor', [1 1 0.3]);

    % Chart 5: Throttle %
    ax_thr = axes('Parent', fig, 'Position', [0.02+4*(strip_w+strip_gap), strip_y, strip_w, strip_h]);
    set(ax_thr, 'Color', chart_colors.bg, 'XColor', [0.5 0.5 0.5], 'YColor', [0.5 0.5 0.5], ...
        'FontSize', 6, 'GridColor', chart_colors.grid, 'GridAlpha', 0.4);
    hold(ax_thr, 'on'); grid(ax_thr, 'on');
    title(ax_thr, 'THR [%]', 'Color', [0.7 0.45 1], 'FontSize', 7, 'FontWeight', 'bold');
    h_strip_thr = plot(ax_thr, NaN, NaN, '-', 'Color', [0.7 0.45 1], 'LineWidth', 1.5);
    ylabel(ax_thr, '%', 'Color', [0.6 0.6 0.6]);

    % Chart 6: Motor RPMs (all motors overlaid)
    ax_rpm = axes('Parent', fig, 'Position', [0.02+5*(strip_w+strip_gap), strip_y, strip_w, strip_h]);
    set(ax_rpm, 'Color', chart_colors.bg, 'XColor', [0.5 0.5 0.5], 'YColor', [0.5 0.5 0.5], ...
        'FontSize', 6, 'GridColor', chart_colors.grid, 'GridAlpha', 0.4);
    hold(ax_rpm, 'on'); grid(ax_rpm, 'on');
    title(ax_rpm, 'RPM', 'Color', [1 0.45 0.65], 'FontSize', 7, 'FontWeight', 'bold');
    rpm_line_colors = [1 0.25 0.3; 0.25 0.85 1; 0.25 0.95 0.45; 1 0.88 0.25; ...
                       0.7 0.45 1; 1 0.45 0.65; 1 0.55 0.15; 0.88 0.88 0.92];
    h_strip_rpm = gobjects(1, n_mot);
    for mi = 1:n_mot
        ci = mod(mi-1, size(rpm_line_colors,1)) + 1;
        h_strip_rpm(mi) = plot(ax_rpm, NaN, NaN, '-', 'Color', [rpm_line_colors(ci,:) 0.7], 'LineWidth', 1.0);
    end
    ylabel(ax_rpm, 'RPM', 'Color', [0.6 0.6 0.6]);

    viz.show_charts = true;  % Toggle with key '5'
    chart_axes = [ax_alt ax_spd ax_att ax_pwr ax_thr ax_rpm];

    %% ============================================================
    %% START TIMER
    %% ============================================================
    sim_timer = timer('ExecutionMode', 'fixedSpacing', ...
        'Period', sim.dt_render, ...
        'TimerFcn', @(~,~) sim_step(), ...
        'ErrorFcn', @(~,e) fprintf('Timer error: %s\n', e.Data.message));
    start(sim_timer);

    fprintf('\n  Drone loaded: %s (%d motors, %.1f kg)\n', dp.frame_type, n_mot, dp.mass);
    fprintf('  Flight mode: %s  |  Press M to switch  |  WASD + SPACE to fly\n\n', ...
        upper(ctrl.flight_mode));
    fprintf('  Location: %s\n', get_active_location_name(env));
    if world.map.is_real
        fprintf('  Map source: OpenStreetMap @ %.6f, %.6f (z%d)\n', ...
            env.map_lat, env.map_lon, env.map_zoom);
    else
        fprintf('  Map source: Procedural fallback (offline-safe)\n');
    end

    %% ============================================================
    %% MAIN LOOP
    %% ============================================================
    function sim_step()
        if ~sim.running || ~isvalid(fig); stop_sim(); return; end

        %% Process keyboard
        ctrl.roll_input = 0; ctrl.pitch_input = 0;
        ctrl.yaw_input = 0; ctrl.throttle_input = 0;
        if keys.a; ctrl.roll_input  = ctrl.roll_input  - 1; end
        if keys.d; ctrl.roll_input  = ctrl.roll_input  + 1; end
        if keys.w; ctrl.pitch_input = ctrl.pitch_input + 1; end
        if keys.s; ctrl.pitch_input = ctrl.pitch_input - 1; end
        if keys.q; ctrl.yaw_input   = ctrl.yaw_input   - 1; end
        if keys.e; ctrl.yaw_input   = ctrl.yaw_input   + 1; end
        if keys.space; ctrl.throttle_input = ctrl.throttle_input + 1; end
        if keys.shift; ctrl.throttle_input = ctrl.throttle_input - 1; end

        %% Wind (Dryden turbulence or simple sinusoidal — attenuated for stability)
        if viz.show_wind
            t_w = sim.time;
            alt_w = max(1, -sim.state(3));
            airspeed = norm(sim.state(4:6));
            if wind_cfg.use_dryden
                wind_cfg.steady_wind = [1.0*sin(0.3*t_w); 0.8*cos(0.2*t_w); 0];
                [viz.wind_vec, ~] = dryden_wind_model(t_w, sim.dt_render, alt_w, ...
                    airspeed, wind_cfg.steady_wind, wind_cfg.turb_intensity);
                viz.wind_vec = viz.wind_vec * 0.5;  % Scale down for realistic hover
            else
                viz.wind_vec = [1.0*sin(0.3*t_w); 0.8*cos(0.2*t_w); 0] + 0.15*randn(3,1);
            end
        else
            viz.wind_vec = [0;0;0];
        end

        %% Physics sub-steps
        n_sub = max(1, round(sim.dt_render / sim.dt_physics));
        for sub = 1:n_sub
            physics_step();
        end

        %% Render + HUD + Instruments
        record_telemetry();
        render_drone();
        update_hud();
        if viz.show_charts; update_strip_charts(); end

        % Push state to Cesium 3D viewer if active
        if cesium.enabled
            lla_c = coord_transforms('ned2lla', sim.state(1:3), ...
                [env.map_lat, env.map_lon, env.map_alt]);
            cs.lat = lla_c(1); cs.lon = lla_c(2); cs.alt = lla_c(3);
            cs.roll = rad2deg(sim.state(7));
            cs.pitch = rad2deg(sim.state(8));
            cs.yaw = mod(rad2deg(sim.state(9)) + 180, 360) - 180;
            cs.speed = norm(sim.state(4:6));
            cs.vz = -sim.state(6);
            cs.soc = batt.soc;
            cs.mode = ctrl.flight_mode;
            cs.time = sim.time;
            cs.location = get_active_location_name(env);
            cesium_bridge('update', cs);
        end

        sim.time = sim.time + sim.dt_render;
    end

    %% ============================================================
    %% PHYSICS
    %% ============================================================
    function physics_step()
        dt  = sim.dt_physics;
        st  = sim.state;
        alt = -st(3);

        switch ctrl.flight_mode
            case 'auto'
                [thrust_cmd, moment_cmds] = auto_flight_controller(st, dt, alt);
            case 'manual'
                [thrust_cmd, moment_cmds] = manual_flight_controller(st);
        end

        %% N-motor mixing
        motor_cmds = mixing_matrix_n(thrust_cmd, moment_cmds, dp);

        %% Motor dynamics (save pre-update speeds for RK4 substep interpolation)
        motor_prev_speeds = sim.motor_speeds;
        if batt.precise
            [sim.motor_speeds, ~, ~, m_current, batt.voltage, ~] = ...
                motor_model_precise(motor_cmds, sim.motor_speeds, dt, dp, batt.soc);
            batt.current = sum(m_current);
        else
            alpha = dt / (dp.tau_motor + dt);
            sim.motor_speeds = sim.motor_speeds + alpha * (motor_cmds - sim.motor_speeds);
            batt.current = sum(dp.kQ * sim.motor_speeds.^2 .* abs(sim.motor_speeds)) ...
                           / (0.85 * dp.V_battery);
            batt.voltage = dp.V_battery;
        end

        %% Dynamics (N-motor generalized) with motor speed interpolation
        % Save pre-update speeds for RK4 substep interpolation
        motor_prev = motor_prev_speeds;  % Speeds at start of step
        motor_new  = sim.motor_speeds;   % Speeds at end of step
        motor_mid  = 0.5 * (motor_prev + motor_new);  % Midpoint estimate

        [state_dot, ~, ~] = multirotor_dynamics(st, motor_prev, viz.wind_vec, dp);

        %% RK4 integration for precision (with motor speed substeps)
        k1 = state_dot;
        [k2,~,~] = multirotor_dynamics(st + 0.5*dt*k1, motor_mid,  viz.wind_vec, dp);
        [k3,~,~] = multirotor_dynamics(st + 0.5*dt*k2, motor_mid,  viz.wind_vec, dp);
        [k4,~,~] = multirotor_dynamics(st + dt*k3,     motor_new,  viz.wind_vec, dp);
        sim.state = st + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);

        %% Wrap yaw angle to [-pi, pi] — CRITICAL for stability
        sim.state(9) = atan2(sin(sim.state(9)), cos(sim.state(9)));

        %% Propeller vibration model (cosmetic only — heavily attenuated)
        % Scale factor 0.02 keeps visual vibration on strip charts
        % without destabilizing hover or flight controllers
        [vib_f, vib_m] = propeller_vibration_model(sim.motor_speeds, layout, dp, sim.time);
        vib_scale = 0.02;
        sim.state(4:6)  = sim.state(4:6)  + vib_scale * (vib_f / dp.mass) * dt;
        sim.state(10:12) = sim.state(10:12) + vib_scale * (dp.I \ vib_m) * dt;

        %% Velocity clamp (prevent runaway speed)
        h_vel = norm(sim.state(4:5));
        if h_vel > ctrl.max_speed
            sim.state(4:5) = sim.state(4:5) * (ctrl.max_speed / h_vel);
        end
        if abs(sim.state(6)) > ctrl.max_climb_rate * 2.5
            sim.state(6) = sign(sim.state(6)) * ctrl.max_climb_rate * 2.5;
        end

        %% Attitude angle safety clamp (prevent full inversion)
        max_angle = deg2rad(45);
        if abs(sim.state(7)) > max_angle
            sim.state(7) = sign(sim.state(7)) * max_angle;
            sim.state(10) = sim.state(10) * 0.5;  % Damp roll rate
        end
        if abs(sim.state(8)) > max_angle
            sim.state(8) = sign(sim.state(8)) * max_angle;
            sim.state(11) = sim.state(11) * 0.5;  % Damp pitch rate
        end
        %% Yaw rate clamp (prevent spin-up)
        max_yaw_rate = deg2rad(360);  % 360 deg/s hard limit
        if abs(sim.state(12)) > max_yaw_rate
            sim.state(12) = sign(sim.state(12)) * max_yaw_rate;
        end

        %% Ground constraint (hard floor — dynamics handles spring-damper)
        if sim.state(3) > 0.001
            sim.state(3) = 0;
            if sim.state(6) > 0; sim.state(6) = -sim.state(6) * 0.3; end  % Bounce
            sim.state(10:12) = sim.state(10:12) * 0.85;  % Angular damping on contact
        end

        %% Battery discharge
        batt.energy_used = batt.energy_used + batt.voltage * batt.current * dt / 3600;
        batt.soc = max(0, 1 - batt.energy_used / dp.energy_Wh);

        %% Battery & motor thermal model
        batt = battery_thermal_model(batt, batt.current, dt, dp);
        % Thermal cutoff: reduce motor authority if overheating
        if batt.thermal_cutoff
            sim.motor_speeds = sim.motor_speeds * 0.5;
        end

        %% Performance tracking (comprehensive & accurate)
        new_pos = sim.state(1:3);
        new_vel = sim.state(4:6);
        new_alt = -sim.state(3);
        new_spd = norm(new_vel);
        climb_v = -new_vel(3);  % Positive = up
        new_tilt = rad2deg(max(abs(sim.state(7)), abs(sim.state(8))));

        % Acceleration & G-force (filtered)
        raw_accel = (new_vel - perf.prev_vel) / dt;
        alpha_filt = 0.15;  % Low-pass filter constant
        perf.accel_smooth = (1-alpha_filt)*perf.accel_smooth + alpha_filt*raw_accel;
        g_vec = perf.accel_smooth + [0; 0; -dp.g];  % Subtract gravity (NED: g is +Z)
        g_force = norm(g_vec) / dp.g;  % In G units (1G = hovering)

        % Vibration index: high-frequency accel magnitude
        perf.vibration = 0.95*perf.vibration + 0.05*norm(raw_accel - perf.accel_smooth);

        % Power
        pwr_now = batt.voltage * batt.current;

        perf.max_alt     = max(perf.max_alt, new_alt);
        perf.max_speed   = max(perf.max_speed, new_spd);
        perf.max_climb   = max(perf.max_climb, climb_v);
        perf.max_descent = max(perf.max_descent, -climb_v);
        perf.max_tilt    = max(perf.max_tilt, new_tilt);
        perf.max_g       = max(perf.max_g, g_force);
        perf.max_power   = max(perf.max_power, pwr_now);

        % Distance: 3D and 2D (ground) — only when moving
        if new_spd > 0.08
            seg3d = norm(new_pos - perf.prev_pos);
            seg2d = norm(new_pos(1:2) - perf.prev_pos(1:2));
            perf.distance    = perf.distance + seg3d;
            perf.distance_2d = perf.distance_2d + seg2d;
        end

        % Airborne & hover time
        if new_alt > 0.15
            perf.flight_time = perf.flight_time + dt;
            if new_spd < 0.5  % Nearly stationary = hovering
                perf.hover_time = perf.hover_time + dt;
            end
        end

        perf.energy_total = batt.energy_used;  % Track for efficiency calc
        perf.prev_pos = new_pos;
        perf.prev_vel = new_vel;
    end

    %% ============================================================
    %% AUTO FLIGHT CONTROLLER (smooth stabilization with filtered PID)
    %% ============================================================
    function [thrust_cmd, moment_cmds] = auto_flight_controller(st, dt, alt)
        euler = st(7:9);
        omega = st(10:12);
        vz_ned = st(6);        % NED: negative = climbing
        climb_rate = -vz_ned;  % positive = up

        % Desired attitude from input
        desired_roll  = ctrl.roll_input  * ctrl.max_tilt;
        desired_pitch = ctrl.pitch_input * ctrl.max_tilt;
        desired_yaw_rate = ctrl.yaw_input * ctrl.max_yaw_rate;

        % Target altitude from throttle input
        climb_rate_cmd = ctrl.throttle_input * ctrl.max_climb_rate;
        ctrl.target_alt = ctrl.target_alt + climb_rate_cmd * dt;
        ctrl.target_alt = max(0, min(80, ctrl.target_alt));

        %% Altitude PID — D-on-measurement (use velocity, not error derivative)
        alt_err = ctrl.target_alt - alt;

        % Integral with anti-windup and decay
        auto.alt_integral = auto.alt_integral + alt_err * dt;
        auto.alt_integral = max(-3, min(3, auto.alt_integral));

        % D-term: use measured climb rate (not d(error)/dt) — smooth, no noise
        % Desired climb rate from altitude error (proportional path)
        desired_climb = alt_err * 1.5;  % Soft P on altitude → climb rate
        desired_climb = max(-cp.alt.max_descent, min(cp.alt.max_climb, desired_climb));
        climb_err = desired_climb - climb_rate;

        % Low-pass filter on D-term for extra smoothness
        alpha_d = dt / (dt + 0.05);  % ~3 Hz cutoff (tau = 0.05s)
        auto.alt_d_filt = (1 - alpha_d) * auto.alt_d_filt + alpha_d * climb_err;

        % Gains scaled to drone mass
        mass_scale = dp.mass / 1.5;
        Kp_a = 4.0 * mass_scale;
        Ki_a = 0.6 * mass_scale;
        Kd_a = 2.5 * mass_scale;

        thrust_correction = Kp_a * alt_err + Ki_a * auto.alt_integral + Kd_a * auto.alt_d_filt;

        % Feed-forward: compensate for tilt (clamped to prevent singularity)
        cos_tilt = max(0.7, cos(euler(1)) * cos(euler(2)));
        thrust_cmd = (dp.mass * dp.g + thrust_correction) / cos_tilt;
        thrust_cmd = max(0, min(dp.max_thrust_total * 0.95, thrust_cmd));

        %% Attitude PD — D-on-measurement (use gyro rates, not error derivative)
        % Gains scaled to inertia
        Kp_roll  = 8.0 * dp.Ixx / 0.0135;
        Kd_roll  = 2.0 * dp.Ixx / 0.0135;
        Kp_pitch = 8.0 * dp.Iyy / 0.0135;
        Kd_pitch = 2.0 * dp.Iyy / 0.0135;

        roll_err  = desired_roll  - euler(1);
        pitch_err = desired_pitch - euler(2);

        % PD with D-on-measurement (uses gyro omega directly)
        tau_x = Kp_roll  * roll_err  - Kd_roll  * omega(1);
        tau_y = Kp_pitch * pitch_err - Kd_pitch * omega(2);
        tau_z = 0.6 * dp.Izz / 0.024 * (desired_yaw_rate - omega(3));

        % Velocity damping — gentle, prevents drift without fighting attitude
        vel_body = euler_to_R_local(euler(1), euler(2), euler(3))' * st(4:6);
        h_speed = norm(st(4:5));

        % Low-pass filter on body velocity for smooth damping
        alpha_v = dt / (dt + 0.1);   % ~1.6 Hz cutoff
        auto.vel_damp_filt = (1 - alpha_v) * auto.vel_damp_filt + alpha_v * vel_body(1:2);

        if ctrl.roll_input == 0 && ctrl.pitch_input == 0
            v_damp = 1.5;   % Moderate braking when sticks centered
        else
            v_damp = 0.4;   % Light damping while flying
        end
        % Progressive braking near speed limit
        if h_speed > ctrl.max_speed * 0.7
            v_damp = v_damp + 3.0 * ((h_speed - ctrl.max_speed*0.7) / (ctrl.max_speed*0.3));
        end
        % Use filtered velocity for damping torques
        tau_y = tau_y + v_damp * auto.vel_damp_filt(1);
        tau_x = tau_x - v_damp * auto.vel_damp_filt(2);

        moment_cmds = [tau_x; tau_y; tau_z];
    end

    %% ============================================================
    %% MANUAL FLIGHT CONTROLLER (game-style with self-level)
    %% ============================================================
    function [thrust_cmd, moment_cmds] = manual_flight_controller(st)
        dt = sim.dt_physics;
        euler = st(7:9); omega = st(10:12);

        % --- Expo curve: softens response near center ---
        expo = ctrl.expo;
        r_in = ctrl.roll_input  * (1-expo) + ctrl.roll_input^3  * expo;
        p_in = ctrl.pitch_input * (1-expo) + ctrl.pitch_input^3 * expo;
        y_in = ctrl.yaw_input   * (1-expo) + ctrl.yaw_input^3   * expo;

        % --- Thrust: hover baseline + climb rate control ---
        hover_thr = dp.mass * dp.g;
        cos_tilt = max(0.7, cos(euler(1)) * cos(euler(2)));

        % Map throttle input to desired climb rate
        desired_vz = ctrl.throttle_input * ctrl.max_climb_rate;
        actual_vz = -st(6);  % positive = climb in NED

        % Smooth climb rate PID (P-only on velocity error)
        vz_err = desired_vz - actual_vz;
        thr_correction = vz_err * dp.mass * 2.5;

        thrust_cmd = (hover_thr + thr_correction) / cos_tilt;
        thrust_cmd = max(0, min(dp.max_thrust_total * 0.85, thrust_cmd));

        if ctrl.self_level
            % --- Self-level mode: sticks command attitude, release = level ---
            max_tilt_man = deg2rad(30);
            desired_roll  = r_in * max_tilt_man;
            desired_pitch = p_in * max_tilt_man;
            desired_yaw_rate = y_in * ctrl.max_yaw_rate;

            % PD attitude controller (D-on-measurement via gyro rates)
            Kp_r = 10.0 * dp.Ixx / 0.0135;
            Kd_r =  3.0 * dp.Ixx / 0.0135;
            Kp_p = 10.0 * dp.Iyy / 0.0135;
            Kd_p =  3.0 * dp.Iyy / 0.0135;

            roll_err  = desired_roll  - euler(1);
            pitch_err = desired_pitch - euler(2);

            tau_x = Kp_r * roll_err  - Kd_r * omega(1);
            tau_y = Kp_p * pitch_err - Kd_p * omega(2);
            tau_z = 1.0 * dp.Izz / 0.024 * (desired_yaw_rate - omega(3));

            % Filtered velocity damping (prevents drift)
            vel_body = euler_to_R_local(euler(1), euler(2), euler(3))' * st(4:6);
            h_speed = norm(st(4:5));
            if ctrl.roll_input == 0 && ctrl.pitch_input == 0
                v_damp = 2.0;
            else
                v_damp = 0.4;
            end
            if h_speed > ctrl.max_speed * 0.7
                v_damp = v_damp + 3.0 * ((h_speed - ctrl.max_speed*0.7) / (ctrl.max_speed*0.3));
            end
            tau_y = tau_y + v_damp * vel_body(1);
            tau_x = tau_x - v_damp * vel_body(2);
        else
            % --- Pure rate mode: sticks command angular rate ---
            max_rate = deg2rad(300);
            desired_roll_rate  = r_in * max_rate;
            desired_pitch_rate = p_in * max_rate;
            desired_yaw_rate   = y_in * ctrl.max_yaw_rate;

            % P-only on rate error (avoids noisy rate-derivative)
            Kp_rate = 0.5 * dp.Ixx / 0.0135;

            rate_err_r = desired_roll_rate  - omega(1);
            rate_err_p = desired_pitch_rate - omega(2);
            rate_err_y = desired_yaw_rate   - omega(3);

            tau_x = Kp_rate * rate_err_r;
            tau_y = Kp_rate * rate_err_p;
            tau_z = 0.8 * dp.Izz / 0.024 * rate_err_y;
        end

        moment_cmds = [tau_x; tau_y; tau_z];
    end

    %% ============================================================
    %% RENDER REALISTIC 3D DRONE (efficient handle-update approach)
    %% ============================================================
    function render_drone()
        if ~isvalid(ax3d); return; end

        pos   = sim.state(1:3);
        eul   = sim.state(7:9);
        alt   = -pos(3);
        L     = dp.arm_length;
        R     = euler_to_R_local(eul(1), eul(2), eul(3));

        %% Advance propeller spin angle based on motor speeds
        omega_max_rad = dp.omega_max * 2*pi/60;
        avg_speed = mean(abs(sim.motor_speeds));
        prop_angle = prop_angle + avg_speed * sim.dt_render * 2.5;
        rpm_norm = min(1, abs(sim.motor_speeds) / omega_max_rad);

        %% --- Update body, top plate, battery ---
        set(drone_h.body, 'Vertices', xfm(geo.body_v, R, pos));
        set(drone_h.top,  'Vertices', xfm(geo.top_v, R, pos));
        set(drone_h.batt, 'Vertices', xfm(geo.batt_v, R, pos));

        %% --- Update arms ---
        for ai = 1:n_mot
            set(drone_h.arms(ai), 'Vertices', xfm(geo.arm_v{ai}, R, pos));
        end

        %% --- Update motor housings ---
        for mi = 1:n_mot
            set(drone_h.motors(mi), 'Vertices', xfm(geo.motor_v{mi}, R, pos));
        end

        %% --- Update spinning propeller blades (airfoil shape) ---
        for mi = 1:n_mot
            mc = geo.motor_centers(mi,:);
            spin_a = prop_angle * layout.spin_dirs(mi);
            for bi = 1:2
                a = spin_a + (bi-1) * pi;
                ca = cos(a); sa = sin(a);
                Rz = [ca -sa 0; sa ca 0; 0 0 1];
                bv = (Rz * geo.blade_template')' + mc;
                set(drone_h.blades(mi,bi), 'Vertices', xfm(bv, R, pos));
            end
        end

        %% --- Update blur discs (opacity from RPM) ---
        for mi = 1:n_mot
            set(drone_h.blur(mi), 'Vertices', xfm(geo.blur_v{mi}, R, pos), ...
                'FaceAlpha', 0.05 + 0.35 * rpm_norm(mi));
        end

        %% --- Update landing gear ---
        for ji = 1:4
            set(drone_h.legs(ji), 'Vertices', xfm(geo.leg_v{ji}, R, pos));
        end
        set(drone_h.skids(1), 'Vertices', xfm(geo.skid_v{1}, R, pos));
        set(drone_h.skids(2), 'Vertices', xfm(geo.skid_v{2}, R, pos));

        %% --- Update forward arrow ---
        fwd_w = xfm(geo.fwd_pts, R, pos);
        set(drone_h.fwd, 'XData', fwd_w(:,1), 'YData', fwd_w(:,2), 'ZData', fwd_w(:,3));

        %% --- Update mode ring ---
        if strcmp(ctrl.flight_mode, 'auto')
            ring_clr = [0 0.8 0.3];
        else
            ring_clr = [1 0.3 0.1];
        end
        ring_w = xfm(geo.ring_pts, R, pos);
        set(drone_h.ring, 'XData', ring_w(:,1), 'YData', ring_w(:,2), ...
            'ZData', ring_w(:,3), 'Color', ring_clr);

        %% --- Update LEDs ---
        for li = 1:n_mot
            set(drone_h.leds(li), 'Vertices', xfm(geo.led_v{li}, R, pos));
        end

        %% Trail (ring buffer — no array reallocation)
        if viz.show_trail
            viz.trail_idx = mod(viz.trail_idx, viz.max_trail) + 1;
            viz.trail_x(viz.trail_idx) = pos(1);
            viz.trail_y(viz.trail_idx) = pos(2);
            viz.trail_z(viz.trail_idx) = alt;
            % Reorder: oldest first, newest last (NaN entries are skipped by MATLAB plot)
            order = [viz.trail_idx+1:viz.max_trail, 1:viz.trail_idx];
            set(h_trail, 'XData', viz.trail_x(order), ...
                'YData', viz.trail_y(order), 'ZData', viz.trail_z(order));
        end

        %% Shadow (realistic: sun-projected, fades with altitude)
        shadow_r = L * 1.4;
        shadow_alpha = max(0.02, 0.25 * min(1, 6/(alt+0.3)));
        % Offset shadow in sun direction (sun at [1 0.4 -0.8])
        sun_offset_x = alt * 0.15;
        sun_offset_y = alt * 0.06;
        % Elongate shadow at high altitude
        shadow_stretch = 1 + alt * 0.02;
        set(h_shadow, 'XData', pos(1)+sun_offset_x+shadow_r*shadow_stretch*cos(shadow_theta), ...
            'YData', pos(2)+sun_offset_y+shadow_r*sin(shadow_theta), ...
            'ZData', ones(1,20)*0.004, 'FaceAlpha', shadow_alpha, ...
            'FaceColor', [0.05 0.05 0.05]);

        %% Altitude reference line
        set(h_alt_line, 'XData', [pos(1) pos(1)], 'YData', [pos(2) pos(2)], 'ZData', [0 alt]);

        %% Wind arrow
        if viz.show_wind
            w = viz.wind_vec;
            set(h_wind, 'XData', pos(1)-5, 'YData', pos(2)-5, 'ZData', alt, ...
                'UData', w(1)*2, 'VData', w(2)*2, 'WData', -w(3)*2, 'Visible', 'on');
        else
            set(h_wind, 'Visible', 'off');
        end

        %% Prop wash dust particles (spawn near ground, driven by motor speed)
        avg_rpm_frac = mean(rpm_norm);
        spawn_chance = avg_rpm_frac * max(0, 1 - alt/3) * 0.6;  % Only near ground
        for di = 1:n_dust
            if dust.life(di) <= 0
                if rand < spawn_chance
                    % Spawn particle under a random motor
                    mi_r = randi(n_mot);
                    mc = geo.motor_centers(mi_r, :);
                    mc_world = xfm(mc, R, pos);
                    dust.pos(di,:) = [mc_world(1) + 0.3*randn, mc_world(2) + 0.3*randn, 0.02];
                    dust.vel(di,:) = [0.8*randn, 0.8*randn, 0.1 + 0.3*rand] * avg_rpm_frac;
                    dust.life(di) = 0.5 + rand * 1.0;
                    dust.alpha(di) = 0.3 + 0.3 * rand;
                end
            else
                dust.life(di) = dust.life(di) - sim.dt_render;
                dust.pos(di,:) = dust.pos(di,:) + dust.vel(di,:) * sim.dt_render;
                dust.vel(di,3) = dust.vel(di,3) - 0.5 * sim.dt_render; % gravity
                dust.pos(di,3) = max(0.01, dust.pos(di,3));
                fade = max(0, dust.life(di) / 1.0);
                set(dust.h(di), 'XData', dust.pos(di,1), 'YData', dust.pos(di,2), ...
                    'ZData', dust.pos(di,3), 'MarkerSize', 2 + 4*(1-fade), ...
                    'Color', [0.75 0.65 0.45 dust.alpha(di)*fade]);
            end
            if dust.life(di) <= 0
                set(dust.h(di), 'XData', NaN, 'YData', NaN, 'ZData', NaN);
            end
        end

        %% Motion ghost trail (afterimage at speed)
        ghost_counter = ghost_counter + 1;
        if ghost_counter >= 3  % Every 3rd frame
            ghost_counter = 0;
            ghost_pos = circshift(ghost_pos, 1, 1);
            ghost_pos(1,:) = [pos(1), pos(2), alt];
        end
        current_speed = norm(sim.state(4:6));
        for gi = 1:n_ghost
            if ~isnan(ghost_pos(gi,1)) && current_speed > 1.5
                set(h_ghost(gi), 'XData', ghost_pos(gi,1), 'YData', ghost_pos(gi,2), ...
                    'ZData', ghost_pos(gi,3));
            else
                set(h_ghost(gi), 'XData', NaN, 'YData', NaN, 'ZData', NaN);
            end
        end

        % Animate dynamic street-level objects (traffic)
        world = update_ground_environment(world, sim.dt_render);

        %% Multi-mode camera system
        if viz.camera_follow
            cam_target = [pos(1), pos(2), alt];
            L_cam = dp.arm_length;
            zm = viz.cam_zoom;  % Distance multiplier from scroll wheel

            switch viz.camera_mode
                case 1  % CHASE CAM — smooth follow behind drone
                    cam_dist = max(2.0, L_cam * 9) * zm;
                    cam_height = alt + max(1.0, L_cam * 3.5) * zm;
                    % Smooth heading to prevent snap on yaw changes
                    yaw_diff = eul(3) - viz.cam_smooth_yaw;
                    yaw_diff = atan2(sin(yaw_diff), cos(yaw_diff)); % wrap to [-pi,pi]
                    viz.cam_smooth_yaw = viz.cam_smooth_yaw + 0.06 * yaw_diff;
                    new_cam_pos = [pos(1) - cam_dist*cos(viz.cam_smooth_yaw), ...
                                   pos(2) - cam_dist*sin(viz.cam_smooth_yaw), ...
                                   cam_height];

                case 2  % FPV CAM — nose-mounted first-person view
                    fwd_offset = L_cam * 1.5;
                    fwd_dir = R * [fwd_offset; 0; 0];
                    new_cam_pos = [pos(1) + fwd_dir(1), ...
                                   pos(2) + fwd_dir(2), ...
                                   alt - fwd_dir(3) + L_cam*0.3];
                    look_dist = L_cam * 12;
                    look_dir = R * [look_dist; 0; 0];
                    cam_target = [pos(1) + look_dir(1), ...
                                  pos(2) + look_dir(2), ...
                                  alt - look_dir(3)];

                case 3  % ORBIT CAM — auto-rotating around drone
                    viz.cam_orbit_angle = viz.cam_orbit_angle + sim.dt_render * 0.3;
                    orbit_dist = max(3.0, L_cam * 12) * zm;
                    orbit_h = alt + max(1.5, L_cam * 5) * zm;
                    new_cam_pos = [pos(1) + orbit_dist*cos(viz.cam_orbit_angle), ...
                                   pos(2) + orbit_dist*sin(viz.cam_orbit_angle), ...
                                   orbit_h];

                case 4  % CINEMATIC CAM — high angle wide shot
                    cine_dist = max(5.0, L_cam * 18) * zm;
                    cine_h = alt + max(4.0, L_cam * 10) * zm;
                    cine_angle = atan2(pos(2), pos(1)) + pi + 0.3;
                    new_cam_pos = [pos(1) + cine_dist*cos(cine_angle), ...
                                   pos(2) + cine_dist*sin(cine_angle), ...
                                   cine_h];

                case 5  % STREET CAM — low-angle chase for street-view feel
                    vel_xy = sim.state(4:5);
                    sp_xy = norm(vel_xy);
                    if sp_xy > 0.5
                        fwd_xy = vel_xy(:)' / sp_xy;
                    else
                        fwd_xy = [cos(eul(3)), sin(eul(3))];
                    end
                    side_xy = [-fwd_xy(2), fwd_xy(1)];
                    st_dist = max(4.5, L_cam * 15) * zm;
                    st_h = max(1.6, min(3.2, alt*0.35 + 1.2));
                    % Prevent cam from going underground
                    st_h = max(0.5, st_h);
                    new_cam_pos = [pos(1) - fwd_xy(1)*st_dist + side_xy(1)*1.0, ...
                                   pos(2) - fwd_xy(2)*st_dist + side_xy(2)*1.0, ...
                                   st_h];
                    cam_target = [pos(1) + fwd_xy(1)*10, ...
                                  pos(2) + fwd_xy(2)*10, ...
                                  max(0.8, alt*0.45)];
            end

            % Smooth camera interpolation (cinematic feel)
            smooth_factor = 0.08;
            if viz.camera_mode == 2
                smooth_factor = 0.22;  % FPV most responsive
            elseif viz.camera_mode == 5
                smooth_factor = 0.14;  % Street cam quick but not jittery
            elseif viz.camera_mode == 1
                smooth_factor = 0.10;  % Chase slightly faster than orbit/cine
            end
            % On first frame after mode switch, snap camera to avoid wild lerp
            if viz.cam_mode_fresh
                viz.cam_smooth_pos = new_cam_pos;
                viz.cam_smooth_tgt = cam_target;
                viz.cam_mode_fresh = false;
            else
                viz.cam_smooth_pos = viz.cam_smooth_pos + smooth_factor * (new_cam_pos - viz.cam_smooth_pos);
                viz.cam_smooth_tgt = viz.cam_smooth_tgt + smooth_factor * (cam_target - viz.cam_smooth_tgt);
            end

            set(ax3d, 'CameraPosition', viz.cam_smooth_pos, ...
                'CameraTarget', viz.cam_smooth_tgt, ...
                'CameraViewAngle', viz.cam_fov);
        end

        %% Dynamic axis limits (per-mode padding)
        switch viz.camera_mode
            case 4;     pad = max(8, L * 25);   % Cinematic: wide
            case 3;     pad = max(6, L * 18);   % Orbit: medium-wide
            case 5;     pad = max(5, L * 16);   % Street: medium
            otherwise;  pad = max(4, L * 15);   % Chase/FPV: tight
        end
        ax3d.XLim = [pos(1)-pad, pos(1)+pad];
        ax3d.YLim = [pos(2)-pad, pos(2)+pad];
        ax3d.ZLim = [-0.5, max(alt+pad, pad*1.5)];

        %% Update 3D HUD overlay (attitude indicator, compass, altimeter)
        update_3d_hud(hud3d, ax3d, sim.state, ctrl, batt, viz);

        drawnow limitrate nocallbacks;
    end

    %% ============================================================
    %% TELEMETRY RECORDING (ring buffer + full log)
    %% ============================================================
    function record_telemetry()
        st  = sim.state;
        alt = -st(3);
        spd = norm(st(4:6));
        vz  = -st(6);
        eul_d = rad2deg(st(7:9));
        eul_d(3) = mod(eul_d(3) + 180, 360) - 180;  % Wrap yaw for charts
        pwr = batt.voltage * batt.current;
        rpm_conv = 60 / (2*pi);

        % G-force from smoothed acceleration
        g_vec = perf.accel_smooth + [0; 0; -dp.g];
        gf = norm(g_vec) / dp.g;

        omega_max_rad = dp.omega_max * 2*pi/60;
        thr = (sum(sim.motor_speeds) / (n_mot * omega_max_rad)) * 100;

        % Update ring buffer
        ts.idx = mod(ts.idx, ts.buf_len) + 1;
        i = ts.idx;
        ts.time(i)    = sim.time;
        ts.alt(i)     = alt;
        ts.alt_tgt(i) = ctrl.target_alt;
        ts.speed(i)   = spd;
        ts.vz(i)      = vz;
        ts.roll(i)    = eul_d(1);
        ts.pitch(i)   = eul_d(2);
        ts.yaw(i)     = eul_d(3);
        ts.power(i)   = pwr;
        ts.gforce(i)  = gf;
        ts.thr_pct(i) = thr;
        ts.mot_rpm(i,:) = (sim.motor_speeds * rpm_conv)';

        % Full flight log (for post-flight analysis)
        if log.idx < log.max_samples
            log.idx = log.idx + 1;
            j = log.idx;
            log.time(j)     = sim.time;
            log.pos(j,:)    = st(1:3)';
            log.vel(j,:)    = st(4:6)';
            log.euler(j,:)  = st(7:9)';
            log.omega(j,:)  = st(10:12)';
            log.mot_rpm(j,:)= (sim.motor_speeds * rpm_conv)';
            log.power(j)    = pwr;
            log.soc(j)      = batt.soc;
            log.gforce(j)   = gf;
            log.alt_tgt(j)  = ctrl.target_alt;
        end
    end

    %% ============================================================
    %% STRIP CHART UPDATE (4 real-time instrument panels)
    %% ============================================================
    function update_strip_charts()
        if ~isvalid(fig); return; end

        % Get ordered time-series data from ring buffer
        if ts.idx < ts.buf_len
            rng_i = 1:ts.idx;
        else
            rng_i = [(ts.idx+1):ts.buf_len, 1:ts.idx];
        end
        t_data = ts.time(rng_i);
        if all(isnan(t_data)); return; end

        t_lim = [max(0, sim.time-15), sim.time+0.5];

        % Altitude chart
        set(h_strip_alt,     'XData', t_data, 'YData', ts.alt(rng_i));
        set(h_strip_alt_tgt, 'XData', t_data, 'YData', ts.alt_tgt(rng_i));
        ax_alt.XLim = t_lim;
        alt_max = max(max(ts.alt(rng_i)), max(ts.alt_tgt(rng_i)));
        if isnan(alt_max) || alt_max < 2; alt_max = 2; end
        ax_alt.YLim = [0, alt_max * 1.2];

        % Speed chart
        set(h_strip_spd, 'XData', t_data, 'YData', ts.speed(rng_i));
        set(h_strip_vz,  'XData', t_data, 'YData', ts.vz(rng_i));
        ax_spd.XLim = t_lim;
        spd_max = max(ts.speed(rng_i));
        if isnan(spd_max) || spd_max < 1; spd_max = 1; end
        vz_data = ts.vz(rng_i);
        ax_spd.YLim = [min(-1, min(vz_data)*1.2), max(spd_max, max(vz_data))*1.3];

        % Attitude chart
        set(h_strip_roll,  'XData', t_data, 'YData', ts.roll(rng_i));
        set(h_strip_pitch, 'XData', t_data, 'YData', ts.pitch(rng_i));
        set(h_strip_yaw,   'XData', t_data, 'YData', ts.yaw(rng_i));
        ax_att.XLim = t_lim;
        att_max = max([abs(ts.roll(rng_i)); abs(ts.pitch(rng_i)); 5]);
        ax_att.YLim = [-att_max*1.3, att_max*1.3];

        % Power & G-force chart
        yyaxis(ax_pwr, 'left');
        set(h_strip_pwr, 'XData', t_data, 'YData', ts.power(rng_i));
        pwr_max = max(ts.power(rng_i));
        if isnan(pwr_max) || pwr_max < 10; pwr_max = 10; end
        ax_pwr.YLim = [0, pwr_max * 1.3];
        yyaxis(ax_pwr, 'right');
        set(h_strip_g, 'XData', t_data, 'YData', ts.gforce(rng_i));
        g_max = max(ts.gforce(rng_i));
        if isnan(g_max) || g_max < 1.5; g_max = 1.5; end
        ax_pwr.YLim = [0, g_max * 1.3];
        ax_pwr.XLim = t_lim;

        % Throttle chart
        set(h_strip_thr, 'XData', t_data, 'YData', ts.thr_pct(rng_i));
        ax_thr.XLim = t_lim;
        ax_thr.YLim = [0, 105];

        % Motor RPM chart
        rpm_data = ts.mot_rpm(rng_i, :);
        for mi_c = 1:n_mot
            set(h_strip_rpm(mi_c), 'XData', t_data, 'YData', rpm_data(:, mi_c));
        end
        ax_rpm.XLim = t_lim;
        rpm_max = max(rpm_data(:));
        if isnan(rpm_max) || rpm_max < 100; rpm_max = 100; end
        ax_rpm.YLim = [0, rpm_max * 1.2];
    end

    %% ============================================================
    %% HUD UPDATE (4 pages)
    %% ============================================================
    function update_hud()
        if ~isvalid(fig); return; end

        st    = sim.state;
        alt   = -st(3);
        speed = norm(st(4:6));
        h_spd = norm(st(4:5));
        eul   = rad2deg(st(7:9));
        eul(3) = mod(eul(3) + 180, 360) - 180;  % Wrap yaw to [-180, 180]
        vz    = -st(6);
        rpm_conv = 60 / (2*pi);
        rpms  = sim.motor_speeds * rpm_conv;
        pwr   = batt.voltage * batt.current;
        g_vec = perf.accel_smooth + [0; 0; -dp.g];
        gf    = norm(g_vec) / dp.g;
        lla_now = coord_transforms('ned2lla', st(1:3), [env.map_lat, env.map_lon, env.map_alt]);
        loc_name = get_active_location_name(env);

        switch viz.hud_page
            case 1  % FLIGHT PAGE
                mode_str = upper(ctrl.flight_mode);
                if strcmp(ctrl.flight_mode, 'auto')
                    mode_str = [mode_str '  [stabilized]'];
                else
                    mode_str = [mode_str '  [rate mode]'];
                end

                if batt.soc > 0.3; batt_warn = '';
                elseif batt.soc > 0.15; batt_warn = ' LOW!';
                else; batt_warn = ' CRITICAL!'; end

                hud_str = sprintf([...
                    '=== FLIGHT DATA ===\n\n', ...
                    'MODE:    %s\n', ...
                    'TIME:    %.1f s\n\n', ...
                    '--- BATTERY ---\n', ...
                    'SOC:     %.0f%%%s\n', ...
                    'Voltage: %.1f V\n', ...
                    'Current: %.1f A\n', ...
                    'Power:   %.0f W\n', ...
                    'Used:    %.2f Wh\n\n', ...
                    '--- POSITION ---\n', ...
                    'X:       %+7.2f m\n', ...
                    'Y:       %+7.2f m\n', ...
                    'ALT:     %7.2f m\n', ...
                    'Target:  %7.2f m\n\n', ...
                    'GPS:     %+.6f\n', ...
                    '         %+.6f\n\n', ...
                    'LOC:     %s\n\n', ...
                    '--- VELOCITY ---\n', ...
                    'H.Speed: %5.2f m/s\n', ...
                    '3DSpeed: %5.2f m/s\n', ...
                    'Climb:  %+6.2f m/s\n\n', ...
                    '--- ATTITUDE ---\n', ...
                    'Roll:   %+6.1f deg\n', ...
                    'Pitch:  %+6.1f deg\n', ...
                    'Yaw:    %+6.1f deg\n', ...
                    'G-Force: %4.2f G\n'], ...
                    mode_str, sim.time, ...
                    batt.soc*100, batt_warn, batt.voltage, batt.current, pwr, batt.energy_used, ...
                    st(1), st(2), alt, ctrl.target_alt, ...
                    lla_now(1), lla_now(2), ...
                    loc_name, ...
                    h_spd, speed, vz, eul(1), eul(2), eul(3), gf);

                if viz.show_wind
                    hud_str = [hud_str, sprintf('\nWIND: [%.1f, %.1f] m/s', ...
                        viz.wind_vec(1), viz.wind_vec(2))];
                end

                % Thermal status
                temp_warn = '';
                if batt.thermal_cutoff
                    temp_warn = ' CUTOFF!';
                elseif batt.thermal_warning
                    temp_warn = ' HOT!';
                end
                hud_str = [hud_str, sprintf('\n\n--- THERMAL ---\nBatt: %.0f°C%s\nMotor: %.0f°C', ...
                    batt.temperature, temp_warn, batt.motor_thermal)];

            case 2  % CONFIG PAGE
                hud_str = sprintf([...
                    '=== DRONE CONFIG ===\n\n', ...
                    'Config:   %s\n', ...
                    'Frame:    %s (%d mot)\n', ...
                    'Mass:     %.3f kg\n', ...
                    'Arm:      %.0f mm\n\n', ...
                    '--- DIMENSIONS ---\n', ...
                    'Body: %.0fx%.0fx%.0f mm\n', ...
                    'Motor: R%.1f H%.1f mm\n', ...
                    'Arm:   %.1fx%.1f mm\n', ...
                    'Gear:  %.0f mm drop\n', ...
                    'Span:  %.0f mm tip2tip\n\n', ...
                    '--- WEIGHT (g) ---\n', ...
                    'Motors:  %3.0f (%dx%.0f)\n', ...
                    'Props:   %3.0f (%dx%.1f)\n', ...
                    'Battery: %3.0f\n', ...
                    'Frame:   %3.0f\n', ...
                    'Other:   %3.0f\n\n', ...
                    '--- PROPULSION ---\n', ...
                    'Prop: %.0f"x%.1f" Kv%d\n', ...
                    'kT: %.2e kQ: %.2e\n', ...
                    'Jp: %.2e kg*m^2\n\n', ...
                    '--- BATTERY ---\n', ...
                    '%s %.1fV  %.1fAh\n', ...
                    '%.0f Wh  %dC  R%.3f\n\n', ...
                    '--- AERO & INERTIA ---\n', ...
                    'Disc Load: %.0f N/m^2\n', ...
                    'Ixx:%.5f Iyy:%.5f\n', ...
                    'Izz:%.5f\n'], ...
                    upper(cfg.config_mode), dp.frame_type, n_mot, ...
                    dp.mass, dp.arm_length * 1000, ...
                    dp.body_width*1000, dp.body_depth*1000, dp.body_height*1000, ...
                    dp.motor_radius*1000, dp.motor_height_dim*1000, ...
                    dp.arm_width*1000, dp.arm_height*1000, ...
                    dp.landing_gear_drop*1000, ...
                    (2*dp.arm_length + dp.d_prop)*1000, ...
                    dp.motor_mass_total*1000, n_mot, dp.motor_mass*1000, ...
                    dp.prop_mass_total*1000, n_mot, dp.prop_mass*1000, ...
                    dp.battery_mass*1000, dp.frame_mass*1000, dp.avionics_mass*1000, ...
                    dp.d_prop/0.0254, dp.d_pitch/0.0254, dp.Kv, ...
                    dp.kT, dp.kQ, dp.Jp, ...
                    dp.battery_type, dp.V_battery, dp.capacity_Ah, ...
                    dp.energy_Wh, dp.battery_C, dp.R_internal, ...
                    dp.disc_loading, ...
                    dp.Ixx, dp.Iyy, dp.Izz);

            case 3  % PERFORMANCE PAGE
                est_remain = 0;
                if batt.current > 0.1
                    est_remain = (batt.soc * dp.capacity_Ah) / batt.current * 60;
                end
                % Specific energy: Wh consumed per km traveled (ground dist)
                if perf.distance_2d > 0.01
                    specific_e = perf.energy_total / (perf.distance_2d / 1000);  % Wh/km
                else
                    specific_e = 0;
                end
                % Average power over flight time
                if perf.flight_time > 0.5
                    avg_pwr = perf.energy_total * 3600 / perf.flight_time;  % Wh->Ws/s = W
                else
                    avg_pwr = 0;
                end

                hud_str = sprintf([...
                    '=== PERFORMANCE ===\n\n', ...
                    '--- DESIGN SPECS ---\n', ...
                    'T/W Ratio:   %.2f:1\n', ...
                    'Hover Power: %.0f W\n', ...
                    'Est Flight:  ~%.0f min\n', ...
                    'Max Speed:   ~%.0f m/s\n\n', ...
                    '--- FLIGHT RECORDS ---\n', ...
                    'Max Alt:     %.2f m\n', ...
                    'Max Speed:   %.2f m/s\n', ...
                    'Max Climb:  +%.2f m/s\n', ...
                    'Max Desc:   -%.2f m/s\n', ...
                    'Max Tilt:    %.1f deg\n', ...
                    'Max G:       %.2f G\n', ...
                    'Peak Power:  %.0f W\n\n', ...
                    '--- EFFICIENCY ---\n', ...
                    'Dist(3D):    %.1f m\n', ...
                    'Dist(gnd):   %.1f m\n', ...
                    'Avg Power:   %.0f W\n', ...
                    'Specific E:  %.1f Wh/km\n', ...
                    'Hover Eff:   %.0f%%\n\n', ...
                    '--- ENDURANCE ---\n', ...
                    'Airborne:    %.0f s\n', ...
                    'Hovering:    %.0f s\n', ...
                    'SOC:         %.1f%%\n', ...
                    'Used:        %.2f Wh\n', ...
                    'Remaining:  ~%.0f min\n', ...
                    'Vibration:   %.3f\n'], ...
                    dp.thrust_to_weight, dp.P_hover, dp.flight_time_min, dp.max_speed, ...
                    perf.max_alt, perf.max_speed, perf.max_climb, perf.max_descent, ...
                    perf.max_tilt, perf.max_g, perf.max_power, ...
                    perf.distance, perf.distance_2d, avg_pwr, specific_e, ...
                    ternary(perf.flight_time > 0, perf.hover_time/perf.flight_time*100, 0), ...
                    perf.flight_time, perf.hover_time, ...
                    batt.soc * 100, batt.energy_used, est_remain, perf.vibration);

            case 4  % MOTORS PAGE
                omega_max_rad = dp.omega_max * 2*pi/60;
                avg_rpm = mean(rpms);
                rpm_spread = max(rpms) - min(rpms);
                % Motor balance: 0% = all same, 100% = huge spread
                if avg_rpm > 10
                    balance = 100 * (1 - rpm_spread / avg_rpm);
                else
                    balance = 100;
                end

                hud_str = sprintf([...
                    '=== MOTOR STATUS ===\n\n']);
                for mi = 1:n_mot
                    pct_i = rpms(mi) / dp.omega_max * 100;
                    bar_i = make_bar(pct_i, 0, 100, 12);
                    dir_s = ternary(layout.spin_dirs(mi) > 0, 'CCW', 'CW ');
                    hud_str = [hud_str, sprintf(...
                        'M%d %s %s %5.0f RPM %3.0f%%\n', ...
                        mi, dir_s, bar_i, rpms(mi), pct_i)]; %#ok<AGROW>
                end
                % Per-motor thrust & torque
                thrusts = dp.kT * sim.motor_speeds.^2;
                torques = dp.kQ * sim.motor_speeds.^2;
                hud_str = [hud_str, sprintf([...
                    '\n--- AGGREGATE ---\n', ...
                    'Avg RPM:     %5.0f\n', ...
                    'RPM Spread:  %5.0f\n', ...
                    'Balance:     %5.1f%%\n', ...
                    'Total Thrust:%.2f N\n', ...
                    'Thrust/Wt:   %.2f\n', ...
                    'Net Torque Z:%.4f Nm\n\n', ...
                    '--- PER MOTOR ---\n'], ...
                    avg_rpm, rpm_spread, balance, ...
                    sum(thrusts), sum(thrusts)/(dp.mass*dp.g), ...
                    sum(layout.spin_dirs .* torques))];
                for mi = 1:n_mot
                    hud_str = [hud_str, sprintf(...
                        'M%d: T=%.2fN Q=%.4fNm\n', ...
                        mi, thrusts(mi), torques(mi))]; %#ok<AGROW>
                end
        end

        set(h_hud, 'String', hud_str);
    end

    %% ============================================================
    %% KEYBOARD
    %% ============================================================
    function key_down(event)
        switch lower(event.Key)
            case 'w';       keys.w = true;
            case 's';       keys.s = true;
            case 'a';       keys.a = true;
            case 'd';       keys.d = true;
            case 'q';       keys.q = true;
            case 'e';       keys.e = true;
            case 'space';   keys.space = true;
            case 'shift';   keys.shift = true;

            case 'h'
                if strcmp(ctrl.flight_mode, 'manual')
                    ctrl.flight_mode = 'auto';
                end
                ctrl.target_alt = -sim.state(3);
                auto.alt_integral = 0;
                fprintf('[AUTO] Hover locked at %.1f m\n', ctrl.target_alt);

            case 'm'
                if strcmp(ctrl.flight_mode, 'auto')
                    ctrl.flight_mode = 'manual';
                    % Reset manual controller state for clean entry
                    manual_ctrl.throttle_hold  = 0;
                    manual_ctrl.prev_roll_err  = 0;
                    manual_ctrl.prev_pitch_err = 0;
                    manual_ctrl.prev_yaw_err   = 0;
                    manual_ctrl.roll_integral  = 0;
                    manual_ctrl.pitch_integral = 0;
                    fprintf('[MODE] MANUAL — Direct rate control. Be careful!\n');
                else
                    ctrl.flight_mode = 'auto';
                    ctrl.target_alt = -sim.state(3);
                    auto.alt_integral = 0;
                    auto.prev_alt_err = 0;
                    auto.prev_roll_err = 0;
                    auto.prev_pitch_err = 0;
                    fprintf('[MODE] AUTO — Stabilized flight. Alt=%.1f m\n', ctrl.target_alt);
                end

            case 'r'
                sim.state = zeros(12,1);
                sim.motor_speeds = ones(n_mot,1) * dp.hover_omega;
                ctrl.target_alt = 0;
                batt.soc = 1.0; batt.energy_used = 0; batt.voltage = dp.V_full;
                auto.alt_integral = 0; auto.prev_alt_err = 0;
                perf.max_alt = 0; perf.max_speed = 0; perf.max_tilt = 0;
                perf.max_climb = 0; perf.max_descent = 0; perf.max_g = 0;
                perf.max_power = 0; perf.distance = 0; perf.distance_2d = 0;
                perf.flight_time = 0; perf.hover_time = 0;
                perf.prev_pos = [0;0;0]; perf.prev_vel = [0;0;0];
                perf.accel_smooth = [0;0;0]; perf.vibration = 0;
                perf.energy_total = 0;
                viz.trail_x(:) = NaN; viz.trail_y(:) = NaN; viz.trail_z(:) = NaN; viz.trail_idx = 0;
                fprintf('[RESET] Drone returned to origin. Battery recharged.\n');

            case '1'
                viz.show_wind = ~viz.show_wind;
                fprintf('Wind: %s\n', onoff(viz.show_wind));

            case '2'
                viz.show_trail = ~viz.show_trail;
                if ~viz.show_trail
                    set(h_trail, 'XData', NaN, 'YData', NaN, 'ZData', NaN);
                    viz.trail_x(:) = NaN; viz.trail_y(:) = NaN; viz.trail_z(:) = NaN; viz.trail_idx = 0;
                end
                fprintf('Trail: %s\n', onoff(viz.show_trail));

            case '3'
                viz.camera_follow = ~viz.camera_follow;
                if ~viz.camera_follow; view(ax3d, 135, 25); end
                fprintf('Camera follow: %s\n', onoff(viz.camera_follow));

            case 'c'
                viz.camera_mode = mod(viz.camera_mode, 5) + 1;
                viz.camera_follow = true;  % Always enable follow on mode switch
                viz.cam_mode_fresh = true; % Snap camera instead of lerp
                cam_names = {'CHASE', 'FPV', 'ORBIT', 'CINEMATIC', 'STREET'};
                fprintf('[CAM] %s mode\n', cam_names{viz.camera_mode});
                if viz.camera_mode == 2
                    viz.cam_fov = 90;  % Wide FPV
                elseif viz.camera_mode == 4
                    viz.cam_fov = 45;  % Narrow cinematic
                elseif viz.camera_mode == 5
                    viz.cam_fov = 78;  % Street-level perspective
                else
                    viz.cam_fov = 60;  % Default
                end

            case 'l'
                apply_location_preset(env.location_idx + 1, false);

            case 't'
                apply_location_preset(env.location_idx, true);

            case 'g'
                lla_now = coord_transforms('ned2lla', sim.state(1:3), ...
                    [env.map_lat, env.map_lon, env.map_alt]);
                fprintf('[GPS] LAT %.6f  LON %.6f  ALT %.1f m ASL\n', ...
                    lla_now(1), lla_now(2), lla_now(3));

            case 'v'
                if ~cesium.enabled
                    cesium.enabled = cesium_bridge('start', env);
                    if cesium.enabled
                        fprintf('[CESIUM] 3D globe viewer launched in browser.\n');
                    else
                        fprintf('[CESIUM] Failed to start viewer.\n');
                    end
                else
                    cesium_bridge('stop');
                    cesium.enabled = false;
                    fprintf('[CESIUM] 3D globe viewer stopped.\n');
                end

            case 'equal'  % '+' key (zoom in)
                viz.cam_fov = max(20, viz.cam_fov - 5);
                fprintf('FOV: %.0f°\n', viz.cam_fov);

            case 'hyphen'  % '-' key (zoom out)
                viz.cam_fov = min(120, viz.cam_fov + 5);
                fprintf('FOV: %.0f°\n', viz.cam_fov);

            case '4'
                batt.precise = ~batt.precise;
                fprintf('Precise battery model: %s\n', onoff(batt.precise));

            case '5'
                viz.show_charts = ~viz.show_charts;
                vis_state = ternary(viz.show_charts, 'on', 'off');
                for ci = 1:length(chart_axes)
                    set(chart_axes(ci), 'Visible', vis_state);
                    set(get(chart_axes(ci), 'Children'), 'Visible', vis_state);
                    set(get(chart_axes(ci), 'Title'), 'Visible', vis_state);
                end
                if viz.show_charts
                    set(ax3d, 'Position', [0.02 0.32 0.70 0.65]);
                else
                    set(ax3d, 'Position', [0.02 0.02 0.70 0.95]);
                end
                fprintf('Strip charts: %s\n', onoff(viz.show_charts));

            case 'tab'
                viz.hud_page = mod(viz.hud_page, 4) + 1;
                pages = {'FLIGHT', 'CONFIG', 'PERFORMANCE', 'MOTORS'};
                fprintf('HUD: %s page\n', pages{viz.hud_page});

            case 'escape'
                sim.running = false;
                stop_sim();
        end
    end

    function key_up(event)
        switch lower(event.Key)
            case 'w';     keys.w = false;
            case 's';     keys.s = false;
            case 'a';     keys.a = false;
            case 'd';     keys.d = false;
            case 'q';     keys.q = false;
            case 'e';     keys.e = false;
            case 'space'; keys.space = false;
            case 'shift'; keys.shift = false;
        end
    end

    function scroll_zoom(event)
        % Scroll up = zoom in (smaller multiplier), scroll down = zoom out
        delta = event.VerticalScrollCount;  % negative = scroll up
        factor = 1.0 + delta * 0.12;
        viz.cam_zoom = max(0.2, min(5.0, viz.cam_zoom * factor));
    end

    function apply_location_preset(target_idx, do_teleport)
        if ~isfield(env, 'location_presets') || isempty(env.location_presets)
            fprintf('[MAP] No location presets available.\n');
            return;
        end

        n_loc = numel(env.location_presets);
        env.location_idx = mod(target_idx - 1, n_loc) + 1;
        loc = env.location_presets(env.location_idx);

        env.map_lat = loc.lat;
        env.map_lon = loc.lon;
        env.map_alt = loc.alt;
        env.map_zoom = loc.zoom;

        world = update_world_map_surface(world, env);

        if do_teleport
            sim.state(1:2) = 0;
            sim.state(4:6) = 0;
            sim.state(10:12) = 0;
            sim.state(3) = min(sim.state(3), -2.0);  % Keep at least 2m above ground
            ctrl.target_alt = max(ctrl.target_alt, 2.0);
            viz.trail_x(:) = NaN; viz.trail_y(:) = NaN; viz.trail_z(:) = NaN; viz.trail_idx = 0;
            ghost_pos(:) = NaN;
            fprintf('[MAP] %s loaded and drone teleported to center (%.6f, %.6f).\n', ...
                loc.name, loc.lat, loc.lon);
        else
            fprintf('[MAP] %s selected at %.6f, %.6f (z%d). Press T to teleport.\n', ...
                loc.name, loc.lat, loc.lon, loc.zoom);
        end
    end

    %% ============================================================
    %% CLEANUP
    %% ============================================================
    function close_sim(~, ~)
        sim.running = false;
        stop_sim();
    end

    function stop_sim()
        try
            if isvalid(sim_timer); stop(sim_timer); delete(sim_timer); end
        catch; end
        if cesium.enabled
            cesium_bridge('stop');
            cesium.enabled = false;
        end
        if isvalid(fig); delete(fig); end

        % Trim log data
        n_log = log.idx;
        if n_log > 10
            flight_log.time    = log.time(1:n_log);
            flight_log.pos     = log.pos(1:n_log,:);
            flight_log.vel     = log.vel(1:n_log,:);
            flight_log.euler   = log.euler(1:n_log,:);
            flight_log.omega   = log.omega(1:n_log,:);
            flight_log.mot_rpm = log.mot_rpm(1:n_log,:);
            flight_log.power   = log.power(1:n_log);
            flight_log.soc     = log.soc(1:n_log);
            flight_log.gforce  = log.gforce(1:n_log);
            flight_log.alt_tgt = log.alt_tgt(1:n_log);
            flight_log.perf    = perf;
            flight_log.dp      = dp;
            flight_log.layout  = layout;
            flight_log.n_mot   = n_mot;

            fprintf('\nFlight ended. Time: %.1f s | Distance: %.0f m | Max alt: %.1f m\n', ...
                sim.time, perf.distance, perf.max_alt);
            fprintf('Generating post-flight analysis...\n');

            % Launch post-flight analysis
            try
                flight_analysis(flight_log);
            catch me
                fprintf('Post-flight analysis error: %s\n', me.message);
            end
        else
            fprintf('\nFlight ended (too short for analysis).\n');
        end
    end

end  % End main function


%% ================================================================
%% LOCAL HELPERS
%% ================================================================
function R = euler_to_R_local(phi, theta, psi)
    cphi = cos(phi); sphi = sin(phi);
    cth = cos(theta); sth = sin(theta);
    cpsi = cos(psi); spsi = sin(psi);
    R = [cth*cpsi, sphi*sth*cpsi-cphi*spsi, cphi*sth*cpsi+sphi*spsi;
         cth*spsi, sphi*sth*spsi+cphi*cpsi, cphi*sth*spsi-sphi*cpsi;
         -sth,     sphi*cth,                 cphi*cth             ];
end


%% ================================================================
%% COORDINATE TRANSFORM: Body frame (NED) -> Plot frame (z-up)
%% ================================================================
function Vp = xfm(Vb, R, pos)
    Vn = (R * Vb')';  % Rotate body->NED world
    Vp = [Vn(:,1)+pos(1), Vn(:,2)+pos(2), -(Vn(:,3)+pos(3))];  % NED->plot
end


%% ================================================================
%% CREATE DRONE GEOMETRY (called once at startup)
%% ================================================================
function [h, g] = create_drone_geometry(ax, layout, dp)
    L = dp.arm_length;
    n_mot = dp.num_motors;
    prop_r = dp.d_prop / 2 * 0.85;
    F6 = [1 2 3 4; 5 6 7 8; 1 2 6 5; 3 4 8 7; 1 4 8 5; 2 3 7 6];

    % Use config dimensions (from drone_config) instead of hardcoded fractions
    bw = dp.body_width;   bd = dp.body_depth;   bh = dp.body_height;

    %% --- Central body plate (carbon fiber dark) ---
    bx = bw; by = bd; bz = bh;
    g.body_v = box_v(bx, by, bz);
    h.body = patch(ax, 'Vertices', g.body_v, 'Faces', F6, ...
        'FaceColor', [0.12 0.12 0.15], 'EdgeColor', [0.22 0.22 0.26], ...
        'FaceLighting', 'gouraud', 'AmbientStrength', 0.35, ...
        'DiffuseStrength', 0.8, 'SpecularStrength', 0.5, ...
        'SpecularExponent', 20, 'BackFaceLighting', 'reverselit');

    %% --- Electronics top plate (PCB green, glossy) ---
    tx = bw*0.65; ty = bd*0.70; tz = bh*0.375;
    g.top_v = box_v(tx, ty, tz);
    g.top_v(:,3) = g.top_v(:,3) - bz - tz;
    h.top = patch(ax, 'Vertices', g.top_v, 'Faces', F6, ...
        'FaceColor', [0.08 0.38 0.08], 'EdgeColor', [0.12 0.45 0.12], ...
        'FaceLighting', 'gouraud', 'AmbientStrength', 0.4, ...
        'DiffuseStrength', 0.6, 'SpecularStrength', 0.45, ...
        'SpecularExponent', 30);

    %% --- Battery pack (underneath body, blue with detail) ---
    btx = bw*0.54; bty = bd*0.40; btz = bh;
    g.batt_v = box_v(btx, bty, btz);
    g.batt_v(:,3) = g.batt_v(:,3) + bz + btz;
    h.batt = patch(ax, 'Vertices', g.batt_v, 'Faces', F6, ...
        'FaceColor', [0.12 0.12 0.50], 'EdgeColor', [0.18 0.18 0.55], ...
        'FaceLighting', 'gouraud', 'AmbientStrength', 0.4, ...
        'DiffuseStrength', 0.7, 'SpecularStrength', 0.3, ...
        'SpecularExponent', 15);

    %% --- Arms (carbon fiber rectangular tubes) ---
    arm_w = dp.arm_width; arm_h = dp.arm_height;
    for i = 1:n_mot
        mp = layout.positions(i,:);
        g.arm_v{i} = tube_v([0 0 0], mp, arm_w, arm_h);
        h.arms(i) = patch(ax, 'Vertices', g.arm_v{i}, 'Faces', F6, ...
            'FaceColor', [0.15 0.15 0.17], 'EdgeColor', [0.25 0.25 0.28], ...
            'FaceLighting', 'gouraud', 'AmbientStrength', 0.35, ...
            'DiffuseStrength', 0.8, 'SpecularStrength', 0.35, ...
            'SpecularExponent', 15);
    end

    %% --- Motor housings (high-poly metallic cylinders) ---
    motor_r = dp.motor_radius; motor_h_val = dp.motor_height_dim; n_seg = 20;
    for i = 1:n_mot
        mp = layout.positions(i,:);
        [cv, cf] = cyl_v(motor_r, motor_h_val, n_seg, mp);
        g.motor_v{i} = cv;
        g.motor_f{i} = cf;
        if layout.spin_dirs(i) < 0
            clr = [0.50 0.06 0.06];   % CW = dark red metallic
        else
            clr = [0.06 0.12 0.50];   % CCW = dark blue metallic
        end
        h.motors(i) = patch(ax, 'Vertices', cv, 'Faces', cf, ...
            'FaceColor', clr, 'EdgeColor', 'none', ...
            'FaceLighting', 'gouraud', 'AmbientStrength', 0.3, ...
            'DiffuseStrength', 0.7, 'SpecularStrength', 0.75, ...
            'SpecularExponent', 40);
    end

    %% --- Propeller blade template (airfoil shape with twist) ---
    % Realistic tapered blade: wide at root, narrow at tip, with pitch twist
    chord_root = prop_r * 0.16;
    chord_mid  = prop_r * 0.13;
    chord_tip  = prop_r * 0.06;
    hub_r      = prop_r * 0.10;
    % 8-point blade outline (airfoil cross-section approximation)
    r_pts = [hub_r, prop_r*0.25, prop_r*0.5, prop_r*0.75, prop_r*0.95, prop_r];
    c_pts = [chord_root, chord_root*0.95, chord_mid, chord_mid*0.8, chord_tip*1.5, chord_tip];
    % Twist: more pitch at root (~15°), less at tip (~5°)
    twist_pts = deg2rad([15, 12, 9, 7, 5.5, 5]);
    % Build blade vertices: leading edge and trailing edge with thickness
    n_blade_pts = length(r_pts);
    blade_v = zeros(n_blade_pts * 2, 3);
    for bp = 1:n_blade_pts
        tw = twist_pts(bp);
        blade_v(bp, :) = [r_pts(bp),  c_pts(bp)/2, -0.003*cos(tw)];  % Leading edge
        blade_v(bp + n_blade_pts, :) = [r_pts(bp), -c_pts(bp)/2, 0.001*cos(tw)];  % Trailing edge
    end
    g.blade_template = blade_v;
    g.blade_faces = [1:n_blade_pts, (2*n_blade_pts):-1:(n_blade_pts+1)];
    g.motor_centers = layout.positions;
    for i = 1:n_mot
        for b = 1:2
            h.blades(i,b) = patch(ax, 'Vertices', g.blade_template, ...
                'Faces', g.blade_faces, ...
                'FaceColor', [0.20 0.20 0.22], 'EdgeColor', [0.12 0.12 0.14], ...
                'FaceLighting', 'gouraud', 'AmbientStrength', 0.35, ...
                'DiffuseStrength', 0.7, 'SpecularStrength', 0.25, ...
                'FaceAlpha', 0.92, 'LineWidth', 0.5);
        end
    end

    %% --- Prop blur discs ---
    n_disc = 24;
    theta_d = linspace(0, 2*pi, n_disc)';
    for i = 1:n_mot
        mp = layout.positions(i,:);
        g.blur_v{i} = [mp(1)+prop_r*cos(theta_d), mp(2)+prop_r*sin(theta_d), ...
                       ones(n_disc,1)*(mp(3)-0.004)];
        if layout.spin_dirs(i) < 0
            blur_clr = [0.8 0.25 0.25];
        else
            blur_clr = [0.25 0.35 0.8];
        end
        h.blur(i) = patch(ax, 'Vertices', g.blur_v{i}, ...
            'Faces', 1:n_disc, ...
            'FaceColor', blur_clr, 'FaceAlpha', 0.05, 'EdgeColor', 'none');
    end

    %% --- Landing gear (4 legs + 2 skids) ---
    leg_drop = dp.landing_gear_drop;
    leg_spread = bw * 0.78;
    body_bot = bz;
    leg_fwd = bw * 0.54;   % Forward attachment offset
    foot_fwd = bw * 0.64;  % Forward foot offset
    att = [ leg_fwd  leg_spread body_bot;
            leg_fwd -leg_spread body_bot;
           -leg_fwd  leg_spread body_bot;
           -leg_fwd -leg_spread body_bot];
    feet = [ foot_fwd  leg_spread*1.3 body_bot+leg_drop;
             foot_fwd -leg_spread*1.3 body_bot+leg_drop;
            -foot_fwd  leg_spread*1.3 body_bot+leg_drop;
            -foot_fwd -leg_spread*1.3 body_bot+leg_drop];
    lw = arm_w*0.27; lh = arm_w*0.27;
    for j = 1:4
        g.leg_v{j} = tube_v(att(j,:), feet(j,:), lw, lh);
        h.legs(j) = patch(ax, 'Vertices', g.leg_v{j}, 'Faces', F6, ...
            'FaceColor', [0.35 0.35 0.35], 'EdgeColor', 'none', ...
            'FaceLighting', 'gouraud', 'AmbientStrength', 0.5);
    end
    sw = arm_w*0.33; sh = arm_w*0.20;
    g.skid_v{1} = tube_v(feet(1,:), feet(2,:), sw, sh);
    g.skid_v{2} = tube_v(feet(3,:), feet(4,:), sw, sh);
    for k = 1:2
        h.skids(k) = patch(ax, 'Vertices', g.skid_v{k}, 'Faces', F6, ...
            'FaceColor', [0.35 0.35 0.35], 'EdgeColor', 'none', ...
            'FaceLighting', 'gouraud', 'AmbientStrength', 0.5);
    end

    %% --- Forward direction arrow ---
    g.fwd_pts = [0 0 0; L*1.5 0 -0.01];
    h.fwd = plot3(ax, [0 0], [0 0], [0 0], ...
        'Color', [0.1 0.85 0.1], 'LineWidth', 2.5);

    %% --- Mode indicator ring ---
    ring_r = max(bw, bd) * 1.25;
    ring_theta = linspace(0, 2*pi, 30)';
    g.ring_pts = [ring_r*cos(ring_theta), ring_r*sin(ring_theta), ...
                  ones(30,1)*(-bz - tz - bh*0.5)];
    h.ring = plot3(ax, zeros(30,1), zeros(30,1), zeros(30,1), ...
        '-', 'Color', [0 0.8 0.3], 'LineWidth', 2);

    %% --- LED indicators (front=white, rear=red) ---
    led_r = motor_r * 0.33;
    led_theta = linspace(0, 2*pi, 8)';
    for i = 1:n_mot
        lp = layout.positions(i,:) * 0.6;
        g.led_v{i} = [lp(1)+led_r*cos(led_theta), ...
                      lp(2)+led_r*sin(led_theta), ...
                      ones(8,1)*(lp(3) - bz - 0.002)];
        if layout.positions(i,1) >= 0
            led_clr = [0.9 0.9 0.9];
        else
            led_clr = [0.9 0.1 0.1];
        end
        h.leds(i) = patch(ax, 'Vertices', g.led_v{i}, ...
            'Faces', 1:8, ...
            'FaceColor', led_clr, 'EdgeColor', 'none', ...
            'FaceAlpha', 0.85, 'FaceLighting', 'none');
    end
end


%% ================================================================
%% 3D GEOMETRY PRIMITIVES
%% ================================================================
function V = box_v(hx, hy, hz)
    V = [ hx  hy -hz; -hx  hy -hz; -hx -hy -hz;  hx -hy -hz;
          hx  hy  hz; -hx  hy  hz; -hx -hy  hz;  hx -hy  hz];
end

function V = tube_v(p1, p2, hw, hh)
    d = p2 - p1; len = norm(d);
    if len < 1e-6; V = box_v(hw, hw, hh); return; end
    dx = d / len;
    if abs(dx(3)) < 0.9; up = [0 0 -1]; else; up = [1 0 0]; end
    side = cross(dx, up); side = side / norm(side);
    up2 = cross(side, dx); up2 = up2 / norm(up2);
    off = [-hw*side - hh*up2;
            hw*side - hh*up2;
            hw*side + hh*up2;
           -hw*side + hh*up2];
    V = [repmat(p1, 4, 1) + off;
         repmat(p2, 4, 1) + off];
end

function [V, F] = cyl_v(r, h, n, center)
    theta = linspace(0, 2*pi, n+1); theta = theta(1:end-1);
    x = r*cos(theta) + center(1);
    y = r*sin(theta) + center(2);
    V = [x' y' ones(n,1)*center(3); x' y' ones(n,1)*(center(3)-h)];
    F = zeros(n, 4);
    for i = 1:n
        j = mod(i, n) + 1;
        F(i,:) = [i j j+n i+n];
    end
end


%% ================================================================
%% ENVIRONMENT
%% ================================================================
function env = default_environment_settings(cfg)
    env.map_lat = 40.748817;     % Manhattan default (Empire State area)
    env.map_lon = -73.985428;
    env.map_alt = 15;
    env.map_zoom = 17;
    env.tile_span = 3;           % 3x3 tile mosaic around center
    env.enable_online_map = true;
    env.enable_city = true;
    env.enable_traffic = true;
    env.enable_clouds = true;
    env.city_extent_m = 180;
    env.min_map_half_m = 120;
    env.max_map_half_m = 260;
    env.map_cache_dir = fullfile(tempdir, 'drone_sim_map_cache');
    env.location_presets = default_location_presets();
    env.location_idx = 1;

    lat_overridden = false;
    lon_overridden = false;
    alt_overridden = false;
    zoom_overridden = false;

    if isfield(cfg, 'map')
        m = cfg.map;
        if isfield(m, 'preset_idx')
            env.location_idx = max(1, min(numel(env.location_presets), round(m.preset_idx)));
        end
        if isfield(m, 'preset_name')
            preset_names = {env.location_presets.name};
            idx = find(strcmpi(m.preset_name, preset_names), 1, 'first');
            if ~isempty(idx)
                env.location_idx = idx;
            end
        end

        if isfield(m, 'lat'); env.map_lat = m.lat; lat_overridden = true; end
        if isfield(m, 'lon'); env.map_lon = m.lon; lon_overridden = true; end
        if isfield(m, 'alt'); env.map_alt = m.alt; alt_overridden = true; end
        if isfield(m, 'zoom'); env.map_zoom = max(12, min(19, round(m.zoom))); zoom_overridden = true; end
        if isfield(m, 'tile_span'); env.tile_span = max(1, min(5, round(m.tile_span))); end
        if isfield(m, 'online'); env.enable_online_map = logical(m.online); end
        if isfield(m, 'enable_city'); env.enable_city = logical(m.enable_city); end
        if isfield(m, 'enable_traffic'); env.enable_traffic = logical(m.enable_traffic); end
        if isfield(m, 'cache_dir'); env.map_cache_dir = m.cache_dir; end
    end

    active_preset = env.location_presets(env.location_idx);
    if ~lat_overridden; env.map_lat = active_preset.lat; end
    if ~lon_overridden; env.map_lon = active_preset.lon; end
    if ~alt_overridden; env.map_alt = active_preset.alt; end
    if ~zoom_overridden; env.map_zoom = active_preset.zoom; end
end

function presets = default_location_presets()
    presets = struct( ...
        'name', {'Midtown NYC', 'San Francisco', 'London', 'Tokyo', 'Dubai', 'Bengaluru'}, ...
        'lat',  {40.748817, 37.774929, 51.507351, 35.689487, 25.204849, 12.971599}, ...
        'lon',  {-73.985428, -122.419418, -0.127758, 139.691711, 55.270782, 77.594566}, ...
        'alt',  {15, 30, 22, 40, 6, 920}, ...
        'zoom', {17, 17, 17, 17, 17, 17});
end

function name = get_active_location_name(env)
    name = 'Custom';
    if isfield(env, 'location_presets') && ~isempty(env.location_presets)
        idx = env.location_idx;
        idx = max(1, min(numel(env.location_presets), idx));
        name = env.location_presets(idx).name;
    end
end

function world = draw_ground(ax, arm_length, env)
    world = struct();
    world.traffic = struct('enabled', false);

    [map_rgb, map_meta] = get_ground_map_texture(env);
    map_rgb = resize_rgb_nn(map_rgb, 420);
    map_rgb = double(map_rgb);
    if max(map_rgb(:)) > 1
        map_rgb = map_rgb / 255;
    end

    map_half = map_meta.half_extent_m;
    [gx, gy] = meshgrid(linspace(-map_half, map_half, size(map_rgb,2)), ...
                        linspace(-map_half, map_half, size(map_rgb,1)));
    gz = zeros(size(gx));
    h_map = surface(ax, gx, gy, gz, map_rgb, 'EdgeColor', 'none', ...
        'FaceLighting', 'gouraud', 'AmbientStrength', 0.78, ...
        'DiffuseStrength', 0.55, 'SpecularStrength', 0.06, 'SpecularExponent', 6);

    % Blend far-field so boundaries are not abrupt when camera pulls back.
    range = max(500, map_half * 2.0);
    fill3(ax, [-range range range -range], [-range -range range range], ...
        [-0.008 -0.008 -0.008 -0.008], [0.24 0.28 0.24], ...
        'EdgeColor', 'none', 'FaceLighting', 'gouraud', 'AmbientStrength', 0.65);

    road_w = max(7, arm_length * 22);
    road_alpha = ternary(map_meta.is_real, 0.38, 0.84);
    fill3(ax, [-map_half map_half map_half -map_half], road_w*[-0.5 -0.5 0.5 0.5], ...
        ones(1,4)*0.010, [0.14 0.14 0.15], 'EdgeColor', 'none', 'FaceAlpha', road_alpha);
    fill3(ax, road_w*[-0.5 -0.5 0.5 0.5], [-map_half -map_half map_half map_half], ...
        ones(1,4)*0.010, [0.14 0.14 0.15], 'EdgeColor', 'none', 'FaceAlpha', road_alpha);

    for d = -map_half:8:map_half
        fill3(ax, [d d+4 d+4 d], 0.16*[-1 -1 1 1], ones(1,4)*0.012, ...
            [0.95 0.92 0.75], 'EdgeColor', 'none', 'FaceAlpha', 0.82);
        fill3(ax, 0.16*[-1 -1 1 1], [d d+4 d+4 d], ones(1,4)*0.012, ...
            [0.95 0.92 0.75], 'EdgeColor', 'none', 'FaceAlpha', 0.82);
    end

    % Helipad retained in center for clear landing reference.
    pad_r = max(2.1, arm_length * 6);
    theta_p = linspace(0, 2*pi, 96);
    fill3(ax, pad_r*cos(theta_p), pad_r*sin(theta_p), ones(1,96)*0.016, ...
        [0.46 0.46 0.49], 'EdgeColor', [0.88 0.80 0.15], 'LineWidth', 2.8, ...
        'FaceAlpha', 0.96, 'FaceLighting', 'gouraud', ...
        'AmbientStrength', 0.58, 'DiffuseStrength', 0.45);
    fill3(ax, pad_r*0.90*cos(theta_p), pad_r*0.90*sin(theta_p), ones(1,96)*0.018, ...
        [0.27 0.27 0.30], 'EdgeColor', 'none', 'FaceAlpha', 0.96);
    hs = pad_r * 0.36;
    fill3(ax, hs*[-0.42 -0.22 -0.22 -0.42], hs*[-0.65 -0.65 0.65 0.65], ...
        ones(1,4)*0.020, [1 1 1], 'EdgeColor', 'none', 'FaceAlpha', 0.92);
    fill3(ax, hs*[0.22 0.42 0.42 0.22], hs*[-0.65 -0.65 0.65 0.65], ...
        ones(1,4)*0.020, [1 1 1], 'EdgeColor', 'none', 'FaceAlpha', 0.92);
    fill3(ax, hs*[-0.22 0.22 0.22 -0.22], hs*[-0.14 -0.14 0.14 0.14], ...
        ones(1,4)*0.020, [1 1 1], 'EdgeColor', 'none', 'FaceAlpha', 0.92);

    % Urban blocks: procedural buildings around the flight area.
    F6 = [1 2 3 4; 5 6 7 8; 1 2 6 5; 3 4 8 7; 1 4 8 5; 2 3 7 6];
    if env.enable_city
        city_lim = min(map_half * 0.90, env.city_extent_m);
        spacing = 20;
        road_clear = road_w * 1.6;
        b_max = 1200;
        b_h = gobjects(b_max, 1);
        bi = 0;

        rng_state = rng;
        seed_val = round(abs(env.map_lat)*1e4 + abs(env.map_lon)*1e4*13 + env.map_zoom*97);
        if seed_val < 1; seed_val = 1; end
        rng(seed_val, 'twister');

        for bx = -city_lim:spacing:city_lim
            for by = -city_lim:spacing:city_lim
                if abs(bx) < road_clear || abs(by) < road_clear
                    continue;
                end
                if norm([bx by]) < max(14, arm_length*34)
                    continue;
                end
                if rand < 0.32
                    continue;
                end

                bw = 7 + 7*rand;
                bd = 7 + 7*rand;
                bh = 8 + 58*(rand^1.45);
                v = box_world_v(bx, by, bw, bd, bh);
                tone = 0.17 + 0.32*rand;
                c = [tone*0.9 tone tone*1.08];

                bi = bi + 1;
                if bi > b_max
                    break;
                end

                b_h(bi) = patch(ax, 'Vertices', v, 'Faces', F6, ...
                    'FaceColor', c, 'EdgeColor', [0.10 0.10 0.12], ...
                    'LineWidth', 0.4, 'FaceLighting', 'gouraud', ...
                    'AmbientStrength', 0.34, 'DiffuseStrength', 0.72, ...
                    'SpecularStrength', 0.22, 'SpecularExponent', 10);
            end
            if bi > b_max
                break;
            end
        end
        rng(rng_state);
        world.city_buildings = b_h(1:bi);
    else
        world.city_buildings = gobjects(0, 1);
    end

    % Street lights and emissive points around main roads.
    city_lim = min(map_half * 0.90, env.city_extent_m);
    for s = -city_lim:20:city_lim
        for side = [-1, 1]
            y1 = side * (road_w*0.65 + 1.8);
            x1 = side * (road_w*0.65 + 1.8);
            plot3(ax, [s s], [y1 y1], [0.02 6.0], '-', 'Color', [0.42 0.42 0.45], 'LineWidth', 1.0);
            plot3(ax, [x1 x1], [s s], [0.02 6.0], '-', 'Color', [0.42 0.42 0.45], 'LineWidth', 1.0);
            plot3(ax, s, y1, 6.0, '.', 'Color', [1.0 0.93 0.72], 'MarkerSize', 11);
            plot3(ax, x1, s, 6.0, '.', 'Color', [1.0 0.93 0.72], 'MarkerSize', 11);
        end
    end

    % Dynamic traffic for street-view realism.
    world.traffic.enabled = env.enable_traffic;
    if world.traffic.enabled
        n_cars = 14;
        world.traffic.h = gobjects(n_cars, 1);
        world.traffic.is_h = false(n_cars, 1);
        world.traffic.lane = zeros(n_cars, 1);
        world.traffic.s = zeros(n_cars, 1);
        world.traffic.v = zeros(n_cars, 1);
        world.traffic.len = zeros(n_cars, 1);
        world.traffic.wid = zeros(n_cars, 1);
        world.traffic.hgt = zeros(n_cars, 1);
        world.traffic.s_min = -city_lim * 0.95;
        world.traffic.s_max = city_lim * 0.95;

        lane_offset = road_w * 0.26;
        lanes = [-lane_offset, lane_offset];
        for i = 1:n_cars
            world.traffic.is_h(i) = mod(i,2) == 1;
            world.traffic.lane(i) = lanes(mod(i-1,2)+1);
            world.traffic.s(i) = world.traffic.s_min + rand * (world.traffic.s_max - world.traffic.s_min);
            dir_sign = ternary(rand > 0.5, 1, -1);
            world.traffic.v(i) = dir_sign * (5 + 9*rand);
            world.traffic.len(i) = 2.8 + 1.9*rand;
            world.traffic.wid(i) = 1.25 + 0.45*rand;
            world.traffic.hgt(i) = 0.85 + 0.35*rand;
            car_clr = 0.18 + 0.74 * rand(1,3);

            if world.traffic.is_h(i)
                cx = world.traffic.s(i);
                cy = world.traffic.lane(i);
                yaw = 0;
            else
                cx = world.traffic.lane(i);
                cy = world.traffic.s(i);
                yaw = pi/2;
            end

            v_car = oriented_box_world_v(cx, cy, world.traffic.len(i), ...
                world.traffic.wid(i), world.traffic.hgt(i), yaw, 0.03);
            world.traffic.h(i) = patch(ax, 'Vertices', v_car, 'Faces', F6, ...
                'FaceColor', car_clr, 'EdgeColor', [0.08 0.08 0.10], ...
                'LineWidth', 0.4, 'FaceLighting', 'gouraud', ...
                'AmbientStrength', 0.36, 'DiffuseStrength', 0.72, ...
                'SpecularStrength', 0.24, 'SpecularExponent', 25);
        end
    end

    mark_d = max(4, arm_length * 9);
    dirs = {'N','E','S','W'};
    dx = [mark_d 0 -mark_d 0];
    dy = [0 mark_d 0 -mark_d];
    for k = 1:4
        text(ax, dx(k), dy(k), 0.05, dirs{k}, 'Color', [0.94 0.92 0.55], ...
            'FontSize', 10, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    end

    h_credit = text(ax, -map_half*0.97, -map_half*0.97, 0.08, ...
        'Map data (c) OpenStreetMap contributors', ...
        'Color', [0.92 0.92 0.92], 'FontSize', 7, ...
        'HorizontalAlignment', 'left', 'FontName', 'Consolas', ...
        'Visible', ternary(map_meta.is_real, 'on', 'off'));

    world.map = map_meta;
    world.map.h_surface = h_map;
    world.map.h_credit = h_credit;
end

function world = update_ground_environment(world, dt)
    if ~isfield(world, 'traffic') || ~isfield(world.traffic, 'enabled') || ~world.traffic.enabled
        return;
    end

    for i = 1:numel(world.traffic.h)
        if ~isvalid(world.traffic.h(i))
            continue;
        end

        world.traffic.s(i) = world.traffic.s(i) + world.traffic.v(i) * dt;
        if world.traffic.s(i) > world.traffic.s_max
            world.traffic.s(i) = world.traffic.s_min;
        elseif world.traffic.s(i) < world.traffic.s_min
            world.traffic.s(i) = world.traffic.s_max;
        end

        if world.traffic.is_h(i)
            cx = world.traffic.s(i);
            cy = world.traffic.lane(i);
            yaw = 0;
            if world.traffic.v(i) < 0
                yaw = pi;
            end
        else
            cx = world.traffic.lane(i);
            cy = world.traffic.s(i);
            yaw = pi/2;
            if world.traffic.v(i) < 0
                yaw = -pi/2;
            end
        end

        v_car = oriented_box_world_v(cx, cy, world.traffic.len(i), ...
            world.traffic.wid(i), world.traffic.hgt(i), yaw, 0.03);
        set(world.traffic.h(i), 'Vertices', v_car);
    end
end

function world = update_world_map_surface(world, env)
    if ~isfield(world, 'map') || ~isfield(world.map, 'h_surface') || ~isvalid(world.map.h_surface)
        return;
    end

    [map_rgb, map_meta] = get_ground_map_texture(env);
    map_rgb = resize_rgb_nn(map_rgb, 420);
    map_rgb = double(map_rgb);
    if max(map_rgb(:)) > 1
        map_rgb = map_rgb / 255;
    end

    map_half = map_meta.half_extent_m;
    [gx, gy] = meshgrid(linspace(-map_half, map_half, size(map_rgb,2)), ...
                        linspace(-map_half, map_half, size(map_rgb,1)));
    gz = zeros(size(gx));
    set(world.map.h_surface, 'XData', gx, 'YData', gy, 'ZData', gz, 'CData', map_rgb);

    if isfield(world.map, 'h_credit') && isvalid(world.map.h_credit)
        set(world.map.h_credit, 'Position', [-map_half*0.97, -map_half*0.97, 0.08]);
        set(world.map.h_credit, 'Visible', ternary(map_meta.is_real, 'on', 'off'));
    end

    map_meta.h_surface = world.map.h_surface;
    if isfield(world.map, 'h_credit')
        map_meta.h_credit = world.map.h_credit;
    end
    world.map = map_meta;
end

function [map_rgb, meta] = get_ground_map_texture(env)
    tile_px = 256;
    n_tile = max(1, env.tile_span);
    img_sz = tile_px * n_tile;
    map_rgb = zeros(img_sz, img_sz, 3, 'uint8');
    loaded_any = false;

    meta = struct();
    meta.is_real = false;
    meta.half_extent_m = 180;

    if env.enable_online_map
        try
            if ~exist(env.map_cache_dir, 'dir')
                mkdir(env.map_cache_dir);
            end

            [x0f, y0f] = latlon_to_tile(env.map_lat, env.map_lon, env.map_zoom);
            x0 = floor(x0f);
            y0 = floor(y0f);
            n = 2^env.map_zoom;
            h = floor(n_tile/2);

            for iy = 1:n_tile
                for ix = 1:n_tile
                    tx = mod(x0 + (ix - h - 1), n);
                    ty = y0 + (iy - h - 1);
                    tile = [];
                    if ty >= 0 && ty < n
                        cache_file = fullfile(env.map_cache_dir, ...
                            sprintf('osm_z%d_x%d_y%d.png', env.map_zoom, tx, ty));
                        tile_url = sprintf('https://tile.openstreetmap.org/%d/%d/%d.png', ...
                            env.map_zoom, tx, ty);
                        tile = get_map_tile_rgb(tile_url, cache_file);
                    end

                    if isempty(tile)
                        tile = zeros(tile_px, tile_px, 3, 'uint8');
                    else
                        loaded_any = true;
                    end

                    r0 = (iy-1)*tile_px + 1;
                    c0 = (ix-1)*tile_px + 1;
                    map_rgb(r0:r0+tile_px-1, c0:c0+tile_px-1, :) = tile;
                end
            end
        catch
            loaded_any = false;
        end
    end

    if ~loaded_any
        map_rgb = fallback_city_texture(img_sz);
        meta.is_real = false;
    else
        bad = map_rgb(:,:,1) == 0 & map_rgb(:,:,2) == 0 & map_rgb(:,:,3) == 0;
        if any(bad(:))
            fb = fallback_city_texture(img_sz);
            for ch = 1:3
                tmp = map_rgb(:,:,ch);
                fbc = fb(:,:,ch);
                tmp(bad) = fbc(bad);
                map_rgb(:,:,ch) = tmp;
            end
        end
        meta.is_real = true;
    end

    mpp = 156543.03392 * cosd(env.map_lat) / (2^env.map_zoom);
    half_m = 0.5 * mpp * img_sz;
    half_m = max(env.min_map_half_m, min(env.max_map_half_m, half_m));
    meta.half_extent_m = half_m;
end

function tile = get_map_tile_rgb(tile_url, cache_file)
    tile = [];
    try
        if ~exist(cache_file, 'file')
            opts = weboptions('Timeout', 4);
            websave(cache_file, tile_url, opts);
        end
        raw = imread(cache_file);

        if ndims(raw) == 3 && size(raw,3) >= 3
            if size(raw,3) == 4
                alpha = double(raw(:,:,4)) / 255;
                rgb = double(raw(:,:,1:3));
                tile = uint8(rgb .* alpha + 255*(1-alpha));
            else
                tile = raw(:,:,1:3);
            end
        end
    catch
        tile = [];
    end
end

function tex = fallback_city_texture(sz)
    [x, y] = meshgrid(linspace(-1, 1, sz), linspace(-1, 1, sz));

    low = 0.48 + 0.08*sin(2*pi*x).*cos(2*pi*y);
    med = 0.18*sin(5.8*pi*x + 1.3).*sin(4.7*pi*y - 0.9);
    hi = 0.07*sin(13*pi*x + 2.2).*cos(11*pi*y + 0.7);
    tex_g = low + med + hi;

    tex = zeros(sz, sz, 3, 'uint8');
    tex(:,:,1) = uint8(255 * max(0, min(1, 0.34 + 0.34*tex_g)));
    tex(:,:,2) = uint8(255 * max(0, min(1, 0.42 + 0.32*tex_g)));
    tex(:,:,3) = uint8(255 * max(0, min(1, 0.34 + 0.30*tex_g)));

    road_band = abs(x) < 0.035 | abs(y) < 0.035;
    for ch = 1:3
        c = tex(:,:,ch);
        c(road_band) = uint8(40 + 8*ch);
        tex(:,:,ch) = c;
    end

    stripe = (abs(y) < 0.0025 & mod(round((x+1)*sz), 34) < 16) | ...
             (abs(x) < 0.0025 & mod(round((y+1)*sz), 34) < 16);
    tex(:,:,1) = uint8(double(tex(:,:,1)) + 150 * stripe);
    tex(:,:,2) = uint8(double(tex(:,:,2)) + 140 * stripe);
    tex(:,:,3) = uint8(double(tex(:,:,3)) + 90 * stripe);
end

function [x_tile, y_tile] = latlon_to_tile(lat_deg, lon_deg, zoom)
    n = 2^zoom;
    lat_rad = deg2rad(lat_deg);
    x_tile = (lon_deg + 180) / 360 * n;
    y_tile = (1 - log(tan(lat_rad) + sec(lat_rad)) / pi) / 2 * n;
end

function V = box_world_v(cx, cy, w, d, h)
    V = oriented_box_world_v(cx, cy, w, d, h, 0, 0);
end

function V = oriented_box_world_v(cx, cy, len, wid, hgt, yaw, z0)
    hx = len * 0.5;
    hy = wid * 0.5;
    p = [ hx  hy;
         -hx  hy;
         -hx -hy;
          hx -hy;
          hx  hy;
         -hx  hy;
         -hx -hy;
          hx -hy];

    R = [cos(yaw) -sin(yaw); sin(yaw) cos(yaw)];
    pr = (R * p')';
    z = [z0 z0 z0 z0 z0+hgt z0+hgt z0+hgt z0+hgt]';
    V = [pr(:,1) + cx, pr(:,2) + cy, z];
end

function img_out = resize_rgb_nn(img_in, max_dim)
    [h, w, ~] = size(img_in);
    if max(h, w) <= max_dim
        img_out = img_in;
        return;
    end

    scale = max_dim / max(h, w);
    h2 = max(2, round(h * scale));
    w2 = max(2, round(w * scale));
    r = round(linspace(1, h, h2));
    c = round(linspace(1, w, w2));
    img_out = img_in(r, c, :);
end


%% ================================================================
%% SKY DOME — Gradient hemisphere for realistic sky
%% ================================================================
function draw_sky_dome(ax, env)
    n_az = 48;
    n_el = 24;
    sky_r = 450;
    az = linspace(0, 2*pi, n_az);
    el = linspace(0, pi/2, n_el);
    [AZ, EL] = meshgrid(az, el);
    X = sky_r * cos(EL) .* cos(AZ);
    Y = sky_r * cos(EL) .* sin(AZ);
    Z = sky_r * sin(EL);

    C = zeros(n_el, n_az, 3);
    for i = 1:n_el
        t = (i-1) / (n_el-1);
        horizon = [0.78 0.83 0.90];
        mid_sky = [0.46 0.64 0.89];
        zenith  = [0.17 0.28 0.61];
        if t < 0.4
            s = t / 0.4;
            clr = horizon * (1-s) + mid_sky * s;
        else
            s = (t - 0.4) / 0.6;
            clr = mid_sky * (1-s) + zenith * s;
        end
        C(i,:,1) = clr(1);
        C(i,:,2) = clr(2);
        C(i,:,3) = clr(3);
    end

    surface(ax, X, Y, Z, C, 'EdgeColor', 'none', 'FaceLighting', 'none', ...
        'FaceAlpha', 0.93);

    sun_az = 0.8;
    sun_el = 0.6;
    sun_r = sky_r * 0.98;
    sun_x = sun_r * cos(sun_el) * cos(sun_az);
    sun_y = sun_r * cos(sun_el) * sin(sun_az);
    sun_z = sun_r * sin(sun_el);
    glow_th = linspace(0, 2*pi, 20);
    fill3(ax, sun_x + 15*cos(glow_th), sun_y + 15*sin(glow_th), ...
        ones(1,20)*sun_z, [1 0.98 0.85], ...
        'EdgeColor', 'none', 'FaceAlpha', 0.58, 'FaceLighting', 'none');
    fill3(ax, sun_x + 6*cos(glow_th), sun_y + 6*sin(glow_th), ...
        ones(1,20)*sun_z, [1 1 0.95], ...
        'EdgeColor', 'none', 'FaceAlpha', 0.86, 'FaceLighting', 'none');

    haze_th = linspace(0, 2*pi, 64);
    haze_r = sky_r * 0.99;
    haze_h = 13;
    haze_x = [haze_r*cos(haze_th); haze_r*cos(haze_th)];
    haze_y = [haze_r*sin(haze_th); haze_r*sin(haze_th)];
    haze_z = [zeros(1,64); ones(1,64)*haze_h];
    haze_c = zeros(2, 64, 3);
    haze_c(1,:,:) = repmat([0.80 0.84 0.86], [64,1]);
    haze_c(2,:,:) = repmat([0.67 0.74 0.84], [64,1]);
    surface(ax, haze_x', haze_y', haze_z', permute(haze_c, [2 1 3]), ...
        'EdgeColor', 'none', 'FaceAlpha', 0.34, 'FaceLighting', 'none');

    if env.enable_clouds
        rng_state = rng;
        rng(12, 'twister');
        for ci = 1:14
            c_r = 18 + 22*rand;
            c_h = 80 + 90*rand;
            c_x = (rand*2 - 1) * 260;
            c_y = (rand*2 - 1) * 260;
            th = linspace(0, 2*pi, 24);
            jit = 1 + 0.22*sin(3*th + rand*2*pi) + 0.12*cos(5*th + rand*2*pi);
            fill3(ax, c_x + c_r*jit.*cos(th), c_y + c_r*0.55*jit.*sin(th), ...
                ones(1,numel(th))*c_h, [0.98 0.98 0.99], ...
                'EdgeColor', 'none', 'FaceAlpha', 0.12 + 0.08*rand, 'FaceLighting', 'none');
        end
        rng(rng_state);
    end
end


%% ================================================================
%% 3D HUD OVERLAY — Attitude indicator, compass, altimeter
%% ================================================================
function hud3d = create_3d_hud(ax)
    % Create persistent HUD graphic handles (updated each frame)
    % These render in screen-space-like coordinates via annotation tricks

    % Attitude indicator crosshair (center of 3D view)
    hud3d.pitch_lines = gobjects(11, 1);
    for i = 1:11
        hud3d.pitch_lines(i) = plot3(ax, [NaN NaN], [NaN NaN], [NaN NaN], ...
            '-', 'Color', [0 1 0.4 0.5], 'LineWidth', 1.0);
    end

    % Horizon reference line
    hud3d.horizon = plot3(ax, [NaN NaN], [NaN NaN], [NaN NaN], ...
        '-', 'Color', [0.2 0.8 1 0.4], 'LineWidth', 1.5);

    % Heading indicator text
    hud3d.heading_txt = text(ax, 0, 0, 0, '', 'Color', [0 1 0.4], ...
        'FontSize', 9, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'bottom', 'FontName', 'Consolas');

    % Altitude bar text
    hud3d.alt_txt = text(ax, 0, 0, 0, '', 'Color', [0.3 0.9 1], ...
        'FontSize', 9, 'FontWeight', 'bold', 'HorizontalAlignment', 'left', ...
        'FontName', 'Consolas');

    % Speed indicator text
    hud3d.spd_txt = text(ax, 0, 0, 0, '', 'Color', [0.3 1 0.5], ...
        'FontSize', 9, 'FontWeight', 'bold', 'HorizontalAlignment', 'right', ...
        'FontName', 'Consolas');

    % Camera mode indicator
    hud3d.cam_txt = text(ax, 0, 0, 0, '', 'Color', [1 0.8 0.2], ...
        'FontSize', 8, 'FontWeight', 'bold', 'HorizontalAlignment', 'right', ...
        'FontName', 'Consolas');

    % Battery bar (color-coded)
    hud3d.batt_txt = text(ax, 0, 0, 0, '', 'Color', [0 1 0.3], ...
        'FontSize', 8, 'FontWeight', 'bold', 'HorizontalAlignment', 'left', ...
        'FontName', 'Consolas');

    % G-force indicator
    hud3d.gforce_txt = text(ax, 0, 0, 0, '', 'Color', [1 1 0.3], ...
        'FontSize', 8, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', ...
        'FontName', 'Consolas');
end

function update_3d_hud(hud3d, ax, state, ctrl, batt, viz)
    if ~isvalid(ax); return; end

    pos = state(1:3);
    alt = -pos(3);
    eul = state(7:9);
    speed = norm(state(4:6));
    heading_deg = mod(rad2deg(eul(3)), 360);

    % Get camera position for screen-relative placement
    cam_pos = get(ax, 'CameraPosition');
    cam_tgt = get(ax, 'CameraTarget');
    cam_dir = cam_tgt - cam_pos;
    cam_dist_val = norm(cam_dir);
    if cam_dist_val < 0.1; return; end
    cam_dir = cam_dir / cam_dist_val;

    % Calculate right and up vectors in camera space
    cam_up = [0 0 1];
    cam_right = cross(cam_dir, cam_up);
    if norm(cam_right) < 0.01; cam_right = [1 0 0]; end
    cam_right = cam_right / norm(cam_right);
    cam_up = cross(cam_right, cam_dir);
    cam_up = cam_up / norm(cam_up);

    % HUD elements placed near camera target in screen space
    hud_center = cam_tgt;
    hud_scale = cam_dist_val * 0.15;  % Scale with camera distance

    % --- Pitch ladder (attitude indicator) ---
    roll_rad = eul(1);
    pitch_deg = rad2deg(eul(2));
    cr = cos(roll_rad); sr = sin(roll_rad);
    for i = 1:11
        pitch_line = (i - 6) * 5;  % -25 to +25 degrees, step 5
        offset = pitch_line * hud_scale * 0.02;
        % Rotated by roll angle
        line_half = hud_scale * 0.3;
        if mod(pitch_line, 10) == 0
            line_half = hud_scale * 0.5;
        end
        p1 = hud_center + (offset * cr) * cam_up + (-line_half * cr - offset * sr) * cam_right ...
             + (offset * sr - line_half * sr) * [0 0 0];
        p2 = hud_center + (offset * cr) * cam_up + (line_half * cr - offset * sr) * cam_right;

        % Simplified: horizontal lines offset vertically by pitch
        y_off = offset * cr;
        x_off = offset * sr;
        lp1 = hud_center - line_half * cam_right + y_off * cam_up - x_off * cam_right;
        lp2 = hud_center + line_half * cam_right + y_off * cam_up + x_off * cam_right;

        if abs(pitch_line - round(pitch_deg/5)*5) <= 2
            set(hud3d.pitch_lines(i), 'XData', [lp1(1) lp2(1)], ...
                'YData', [lp1(2) lp2(2)], 'ZData', [lp1(3) lp2(3)], ...
                'Color', [0 1 0.4 0.6], 'LineWidth', 1.2);
        else
            set(hud3d.pitch_lines(i), 'XData', [lp1(1) lp2(1)], ...
                'YData', [lp1(2) lp2(2)], 'ZData', [lp1(3) lp2(3)], ...
                'Color', [0 1 0.4 0.25], 'LineWidth', 0.7);
        end
    end

    % --- Horizon line ---
    hz_half = hud_scale * 1.2;
    hz1 = hud_center - hz_half * cam_right;
    hz2 = hud_center + hz_half * cam_right;
    set(hud3d.horizon, 'XData', [hz1(1) hz2(1)], ...
        'YData', [hz1(2) hz2(2)], 'ZData', [hz1(3) hz2(3)]);

    % --- Heading text (top center) ---
    hdg_pos = hud_center + hud_scale * 0.9 * cam_up;
    cardinal = heading_to_cardinal(heading_deg);
    set(hud3d.heading_txt, 'Position', hdg_pos, ...
        'String', sprintf('HDG %03.0f %s', heading_deg, cardinal));

    % --- Altitude text (right side) ---
    alt_pos = hud_center + hud_scale * 0.8 * cam_right + hud_scale * 0.1 * cam_up;
    set(hud3d.alt_txt, 'Position', alt_pos, ...
        'String', sprintf('ALT\n%.1fm\nTGT\n%.1fm', alt, ctrl.target_alt));

    % --- Speed text (left side) ---
    spd_pos = hud_center - hud_scale * 0.8 * cam_right + hud_scale * 0.1 * cam_up;
    set(hud3d.spd_txt, 'Position', spd_pos, ...
        'String', sprintf('SPD\n%.1f\nm/s', speed));

    % --- Camera mode (top right) ---
    cam_names = {'CHASE', 'FPV', 'ORBIT', 'CINE', 'STREET'};
    cr_pos = hud_center + hud_scale * 0.85 * cam_right + hud_scale * 0.85 * cam_up;
    set(hud3d.cam_txt, 'Position', cr_pos, ...
        'String', sprintf('CAM:%s', cam_names{viz.camera_mode}));

    % --- Battery (top left) ---
    if batt.soc > 0.3
        batt_clr = [0 1 0.3];
    elseif batt.soc > 0.15
        batt_clr = [1 0.8 0];
    else
        batt_clr = [1 0.2 0.1];
    end
    bl_pos = hud_center - hud_scale * 0.85 * cam_right + hud_scale * 0.85 * cam_up;
    set(hud3d.batt_txt, 'Position', bl_pos, ...
        'String', sprintf('BAT %.0f%%', batt.soc*100), 'Color', batt_clr);

    % --- G-force (bottom center) ---
    g_vec = [0; 0; -9.81];
    gf = norm(state(4:6)) / 9.81;  % Simplified for HUD display
    gf_pos = hud_center - hud_scale * 0.7 * cam_up;
    set(hud3d.gforce_txt, 'Position', gf_pos, ...
        'String', sprintf('%.1fG', max(0.1, 1.0)));
end

function s = heading_to_cardinal(hdg)
    dirs = {'N','NE','E','SE','S','SW','W','NW'};
    idx = round(hdg / 45);
    idx = mod(idx, 8) + 1;
    s = dirs{idx};
end


%% ================================================================
%% HUD UTILITIES
%% ================================================================
function bar = make_bar(val, vmin, vmax, width)
    frac = (val - vmin) / (vmax - vmin);
    frac = max(0, min(1, frac));
    filled = round(frac * width);
    bar = ['[', repmat('|', 1, filled), repmat('.', 1, width - filled), ']'];
end

function s = onoff(b)
    if b; s = 'ON'; else; s = 'OFF'; end
end

function r = ternary(cond, a, b)
    if cond; r = a; else; r = b; end
end
