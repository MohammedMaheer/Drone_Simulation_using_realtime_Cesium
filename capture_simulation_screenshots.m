function capture_simulation_screenshots()
% CAPTURE_SIMULATION_SCREENSHOTS
%   Runs a representative mission and saves PNG screenshots of:
%     fig9_dashboard_takeoff.png       — live dashboard ~5  s into mission
%     fig10_dashboard_cruise.png       — live dashboard ~25 s into mission
%     fig11_dashboard_complete.png     — live dashboard at end of mission
%     fig12_post_flight_overview.png   — plot_flight_data(...) output
%     fig13_3d_trajectory.png          — 3-D ground/air track
%   All written to paper_figures/ at 200 dpi, black-text-on-white.

    out_dir = fullfile(pwd, 'paper_figures');
    if ~exist(out_dir, 'dir'); mkdir(out_dir); end

    % ---- enforce light theme everywhere -----------------------------------
    set(groot, 'defaultFigureColor',   'w');
    set(groot, 'defaultAxesColor',     'w');
    set(groot, 'defaultAxesXColor',    'k');
    set(groot, 'defaultAxesYColor',    'k');
    set(groot, 'defaultAxesZColor',    'k');
    set(groot, 'defaultTextColor',     'k');
    set(groot, 'defaultAxesGridColor', [0.15 0.15 0.15]);
    set(groot, 'defaultLegendTextColor', 'k');
    set(groot, 'defaultLegendEdgeColor', 'k');
    set(groot, 'defaultLegendColor',     'w');

    % ---- mission setup ----------------------------------------------------
    dp    = drone_params();
    cp    = controller_params();
    sp    = sensor_params();
    sim_p = sim_params();

    clear attitude_controller position_controller altitude_controller
    clear flight_controller waypoint_manager telemetry_dashboard

    mission_params.altitude = sim_p.scenario.target_alt;
    mission_params.size     = 20;
    waypoints = mission_profiles('square', mission_params);

    wp_params.accept_radius = 2.0;
    wp_params.accept_alt    = 1.5;
    wp_params.loiter_time   = 2.0;

    dt    = sim_p.dt;
    t_end = 60;
    N     = round(t_end / dt);
    log_interval  = round(sim_p.dt_log / dt);
    dash_interval = round(sim_p.dt_viz / dt);

    state = [sim_p.init.position; sim_p.init.velocity; ...
             sim_p.init.euler;    sim_p.init.omega];
    motor_speeds = ones(4,1) * 100;
    batt_soc = 1.0;
    logger   = telemetry_logger(ceil(N/log_interval) + 100);

    capture_times = struct( ...
        'takeoff',  5, ...
        'cruise',  25, ...
        'complete', t_end - 1);
    captured = struct('takeoff', false, 'cruise', false, 'complete', false);

    fprintf('Running square mission with dashboard capture ...\n');
    tic;
    for i = 1:N
        t = (i-1) * dt;

        wind = [0;0;0];
        [target, wp_status] = waypoint_manager(state(1:3), waypoints, wp_params);
        [thrust_cmd, moment_cmds, ctrl_data] = flight_controller(state, target, dp, cp);
        motor_cmds = mixing_matrix(thrust_cmd, moment_cmds, dp);
        [motor_speeds, ~, ~, current] = motor_model(motor_cmds, motor_speeds, dt, dp);
        [state_dot, ~, ~] = quadrotor_dynamics(state, motor_speeds, wind, dp);
        state = state + state_dot * dt;
        batt_soc = max(0, batt_soc - sum(current)*dt/(dp.capacity_Ah*3600));

        if mod(i, log_interval) == 0
            logger.log(t, state, motor_speeds, ctrl_data, [], [], batt_soc, wp_status, wind);
        end
        if mod(i, dash_interval) == 0
            telemetry_dashboard(state, ctrl_data, batt_soc, wp_status, t);
        end

        % screenshot at key times
        if ~captured.takeoff && t >= capture_times.takeoff
            grab_dashboard(out_dir, 'fig9_dashboard_takeoff.png');
            captured.takeoff = true;
        end
        if ~captured.cruise && t >= capture_times.cruise
            grab_dashboard(out_dir, 'fig10_dashboard_cruise.png');
            captured.cruise = true;
        end
        if ~captured.complete && t >= capture_times.complete
            grab_dashboard(out_dir, 'fig11_dashboard_complete.png');
            captured.complete = true;
        end
    end
    fprintf('Sim done in %.1f s.\n', toc);

    flight_data = logger.get_data();

    % ---- post-flight overview --------------------------------------------
    plot_flight_data(flight_data);
    fh = gcf;
    blacken(fh);
    exportgraphics(fh, fullfile(out_dir,'fig12_post_flight_overview.png'), ...
        'Resolution', 200, 'BackgroundColor','white');
    fprintf('  wrote fig12_post_flight_overview.png\n');
    close(fh);

    % ---- 3-D trajectory ---------------------------------------------------
    pos = flight_data.position;
    fh = figure('Name','3D Track','Color','w','Position',[200 200 900 700]);
    plot3(pos(:,1), pos(:,2), -pos(:,3), 'b-', 'LineWidth', 1.4); hold on;
    plot3(pos(1,1), pos(1,2), -pos(1,3), 'go', 'MarkerSize', 10, 'MarkerFaceColor','g');
    plot3(pos(end,1), pos(end,2), -pos(end,3), 'rs', 'MarkerSize', 10, 'MarkerFaceColor','r');
    plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3), 'k--o', ...
        'MarkerFaceColor','y','LineWidth',1.0);
    grid on; axis equal; box on;
    xlabel('X North [m]'); ylabel('Y East [m]'); zlabel('Altitude [m]');
    title('3-D Mission Trajectory (square pattern, 20 m, 60 s)');
    legend('Flown path','Start','End','Waypoints','Location','northeast');
    view(35, 30);
    blacken(fh);
    exportgraphics(fh, fullfile(out_dir,'fig13_3d_trajectory.png'), ...
        'Resolution', 200, 'BackgroundColor','white');
    fprintf('  wrote fig13_3d_trajectory.png\n');
    close(fh);

    fprintf('\nAll screenshots written to %s\n', out_dir);
end

% =========================================================================
function grab_dashboard(out_dir, fname)
    h = findall(0, 'Type', 'figure', 'Name', 'Drone Telemetry Dashboard');
    if isempty(h)
        fprintf('  [warn] dashboard figure not found, skipping %s\n', fname);
        return
    end
    fh = h(1);
    drawnow;
    blacken(fh);
    exportgraphics(fh, fullfile(out_dir, fname), ...
        'Resolution', 200, 'BackgroundColor','white');
    fprintf('  wrote %s\n', fname);
end

function blacken(fh)
    set(fh, 'Color', 'w');
    axs = findall(fh, 'Type', 'axes');
    for k = 1:numel(axs)
        ax = axs(k);
        set(ax, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'ZColor', 'k', ...
                'GridColor', [0.15 0.15 0.15]);
        try set(ax.Title,  'Color','k'); catch; end
        try set(ax.XLabel, 'Color','k'); catch; end
        try set(ax.YLabel, 'Color','k'); catch; end
        try set(ax.ZLabel, 'Color','k'); catch; end
    end
    set(findall(fh, 'Type', 'text'), 'Color', 'k');
    legs = findall(fh, 'Type', 'legend');
    for k = 1:numel(legs)
        set(legs(k), 'TextColor', 'k', 'EdgeColor', 'k', 'Color', 'w');
    end
end
