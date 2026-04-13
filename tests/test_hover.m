function test_hover()
% TEST_HOVER  Test hover stability at a fixed altitude.
%
%   Verifies the drone can take off and maintain a stable hover at 10m
%   with minimal position drift and attitude oscillation.

    fprintf('=== TEST: Hover Stability ===\n');

    %% Initialize
    dp = drone_params();
    cp = controller_params();
    sp = sensor_params();
    sim_p = sim_params();

    % Clear persistent variables
    clear attitude_controller position_controller altitude_controller
    clear flight_controller imu_model gps_model barometer_model
    clear magnetometer_model state_estimator waypoint_manager

    %% Target
    target.position = [0; 0; -10];  % 10m altitude (NED)
    target.yaw = 0;

    %% Simulation
    dt = sim_p.dt;
    t_end = 30;  % 30 seconds for hover test
    N = round(t_end / dt);

    state = zeros(12, 1);
    motor_speeds = ones(4,1) * dp.hover_omega;
    wind = [0; 0; 0];

    logger = telemetry_logger(N);
    batt_soc = 1.0;

    fprintf('Running hover test for %.0f seconds...\n', t_end);

    for i = 1:N
        t = (i-1) * dt;

        % Controller
        [thrust_cmd, moment_cmds, ctrl_data] = flight_controller(state, target, dp, cp);

        % Mixing
        motor_cmds = mixing_matrix(thrust_cmd, moment_cmds, dp);

        % Motor dynamics
        [motor_speeds, ~, ~, current] = motor_model(motor_cmds, motor_speeds, dt, dp);

        % Plant dynamics
        [state_dot, ~, ~] = quadrotor_dynamics(state, motor_speeds, wind, dp);
        state = state + state_dot * dt;

        % Battery
        total_current = sum(current);
        batt_soc = batt_soc - total_current * dt / (dp.capacity_Ah * 3600);

        % Log
        if mod(i, 10) == 0
            logger.log(t, state, motor_speeds, ctrl_data, [], [], batt_soc, [], wind);
        end
    end

    %% Analyze Results
    data = logger.get_data();
    alt_final = -data.position(end, 3);
    pos_drift = norm(data.position(end, 1:2));
    max_roll  = max(abs(rad2deg(data.euler(:,1))));
    max_pitch = max(abs(rad2deg(data.euler(:,2))));

    fprintf('\n--- Hover Test Results ---\n');
    fprintf('Final Altitude:    %.2f m (target: 10.00 m)\n', alt_final);
    fprintf('Position Drift:    %.3f m\n', pos_drift);
    fprintf('Max Roll:          %.2f deg\n', max_roll);
    fprintf('Max Pitch:         %.2f deg\n', max_pitch);
    fprintf('Final Battery:     %.1f%%\n', batt_soc * 100);

    % Pass/Fail criteria
    alt_err = abs(alt_final - 10);
    if alt_err < 0.5 && pos_drift < 1.0 && max_roll < 15 && max_pitch < 15
        fprintf('RESULT: PASS\n\n');
    else
        fprintf('RESULT: FAIL\n\n');
    end

    % Plot
    plot_flight_data(data);

end
