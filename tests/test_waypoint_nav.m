function test_waypoint_nav()
% TEST_WAYPOINT_NAV  Test waypoint navigation with a square mission profile.
%
%   Validates the drone can navigate through a sequence of waypoints
%   using the position controller and waypoint manager.

    fprintf('=== TEST: Waypoint Navigation ===\n');

    %% Initialize
    dp = drone_params();
    cp = controller_params();
    sp = sensor_params();
    sim_p = sim_params();

    clear attitude_controller position_controller altitude_controller
    clear flight_controller waypoint_manager imu_model gps_model
    clear barometer_model magnetometer_model state_estimator

    %% Define Mission
    mission_params.altitude = 10;
    mission_params.size = 15;
    waypoints = mission_profiles('square', mission_params);
    fprintf('Mission: Square pattern with %d waypoints\n', size(waypoints,1));

    wp_params.accept_radius = 2.0;
    wp_params.accept_alt    = 1.5;
    wp_params.loiter_time   = 1.0;

    %% Simulation
    dt = sim_p.dt;
    t_end = 90;  % Longer sim for navigation
    N = round(t_end / dt);

    state = zeros(12, 1);
    motor_speeds = ones(4,1) * dp.hover_omega * 0.5;
    wind = [0.5; 0.3; 0];

    logger = telemetry_logger(N);
    batt_soc = 1.0;

    fprintf('Running waypoint nav test for %.0f seconds...\n', t_end);

    for i = 1:N
        t = (i-1) * dt;

        % Waypoint manager
        [target, wp_status] = waypoint_manager(state(1:3), waypoints, wp_params);

        % Controller
        [thrust_cmd, moment_cmds, ctrl_data] = flight_controller(state, target, dp, cp);

        % Mixing & Motors
        motor_cmds = mixing_matrix(thrust_cmd, moment_cmds, dp);
        [motor_speeds, ~, ~, current] = motor_model(motor_cmds, motor_speeds, dt, dp);

        % Plant
        [state_dot, ~, ~] = quadrotor_dynamics(state, motor_speeds, wind, dp);
        state = state + state_dot * dt;

        % Battery
        batt_soc = batt_soc - sum(current) * dt / (dp.capacity_Ah * 3600);

        % Log
        if mod(i, 10) == 0
            logger.log(t, state, motor_speeds, ctrl_data, [], [], batt_soc, wp_status, wind);
        end

        % Early exit if mission complete
        if wp_status.completed && t > 20
            fprintf('Mission completed at t=%.1f s\n', t);
            break;
        end
    end

    %% Results
    data = logger.get_data();

    fprintf('\n--- Waypoint Nav Results ---\n');
    fprintf('Waypoints Reached: %d / %d\n', wp_status.current_idx, wp_status.total);
    fprintf('Mission Complete:  %s\n', ternary(wp_status.completed, 'YES', 'NO'));
    fprintf('Max Speed:         %.2f m/s\n', max(sqrt(sum(data.velocity.^2, 2))));
    fprintf('Final Battery:     %.1f%%\n', batt_soc * 100);

    if wp_status.completed
        fprintf('RESULT: PASS\n\n');
    else
        fprintf('RESULT: FAIL\n\n');
    end

    % Plot with waypoints overlay
    figure('Name', 'Waypoint Navigation', 'Position', [100 100 800 600]);
    plot(data.position(:,1), data.position(:,2), 'b-', 'LineWidth', 1.5);
    hold on;
    plot(waypoints(:,1), waypoints(:,2), 'r*-', 'MarkerSize', 12, 'LineWidth', 1);
    plot(data.position(1,1), data.position(1,2), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
    grid on; axis equal;
    xlabel('X North [m]'); ylabel('Y East [m]');
    title('Waypoint Navigation Test');
    legend('Actual Path', 'Waypoints', 'Start');

    post_flight_analysis(data);

end


function result = ternary(cond, a, b)
    if cond; result = a; else; result = b; end
end
