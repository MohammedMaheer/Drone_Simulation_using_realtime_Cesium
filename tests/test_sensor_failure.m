function test_sensor_failure()
% TEST_SENSOR_FAILURE  Test state estimator behavior under sensor degradation.
%
%   Simulates GPS outage at t=15s and verifies the estimator holds 
%   reasonable estimates using IMU + barometer only.

    fprintf('=== TEST: Sensor Failure (GPS Outage) ===\n');

    %% Initialize
    dp = drone_params();
    cp = controller_params();
    sp = sensor_params();
    sim_p = sim_params();

    clear attitude_controller position_controller altitude_controller
    clear flight_controller imu_model gps_model barometer_model
    clear magnetometer_model state_estimator

    %% Target
    target.position = [0; 0; -10];
    target.yaw = 0;

    %% Simulation
    dt = sim_p.dt;
    t_end = 40;
    N = round(t_end / dt);

    state = zeros(12, 1);
    motor_speeds = ones(4,1) * dp.hover_omega;
    wind = [0.5; 0.2; 0];

    logger = telemetry_logger(N);
    batt_soc = 1.0;

    fprintf('Running sensor failure test for %.0f seconds...\n', t_end);
    fprintf('GPS outage at t=15s, recovery at t=30s\n');

    for i = 1:N
        t = (i-1) * dt;

        % Controller
        [thrust_cmd, moment_cmds, ctrl_data] = flight_controller(state, target, dp, cp);
        motor_cmds = mixing_matrix(thrust_cmd, moment_cmds, dp);
        [motor_speeds, ~, ~, current] = motor_model(motor_cmds, motor_speeds, dt, dp);

        % Plant
        [state_dot, ~, ~] = quadrotor_dynamics(state, motor_speeds, wind, dp);
        state = state + state_dot * dt;

        % -- Sensors --
        % IMU (always available)
        R = euler_to_dcm(state(7), state(8), state(9));
        accel_true = R' * [0; 0; -dp.g];
        [accel_meas, gyro_meas] = imu_model(accel_true, state(10:12), dt, sp);

        % GPS (outage between 15-30s)
        gps_available = ~(t >= 15 && t < 30);
        if gps_available
            [gps_pos, gps_vel, gps_valid] = gps_model(state(1:3), state(4:6), t, sp);
        else
            gps_pos = [0;0;0];
            gps_vel = [0;0;0];
            gps_valid = false;
        end

        % Barometer
        baro_alt = barometer_model(-state(3), t, sp);

        % Magnetometer
        mag_heading = magnetometer_model(state(7:9), sp);

        % Build sensor struct
        sensor_data.accel = accel_meas;
        sensor_data.gyro = gyro_meas;
        sensor_data.gps_pos = gps_pos;
        sensor_data.gps_vel = gps_vel;
        sensor_data.gps_valid = gps_valid;
        sensor_data.baro_alt = baro_alt;
        sensor_data.mag_heading = mag_heading;

        % State estimator
        [est_state, ~] = state_estimator(sensor_data, dt, sp, dp);

        batt_soc = batt_soc - sum(current) * dt / (dp.capacity_Ah * 3600);

        if mod(i, 10) == 0
            logger.log(t, state, motor_speeds, ctrl_data, sensor_data, est_state, batt_soc, [], wind);
        end
    end

    %% Results
    data = logger.get_data();

    % Compare true vs estimated during GPS outage
    outage_idx = data.time >= 15 & data.time < 30;
    pos_err_outage = data.position(outage_idx, :) - data.est_pos(outage_idx, :);
    max_est_error = max(sqrt(sum(pos_err_outage.^2, 2)));

    % After GPS recovery
    recovery_idx = data.time >= 30;
    if any(recovery_idx)
        pos_err_recovery = data.position(recovery_idx,:) - data.est_pos(recovery_idx,:);
        final_est_error = sqrt(sum(pos_err_recovery(end,:).^2));
    else
        final_est_error = NaN;
    end

    fprintf('\n--- Sensor Failure Results ---\n');
    fprintf('Max estimation error during GPS outage: %.2f m\n', max_est_error);
    fprintf('Final estimation error after recovery:  %.2f m\n', final_est_error);

    if max_est_error < 20 && final_est_error < 3
        fprintf('RESULT: PASS\n\n');
    else
        fprintf('RESULT: FAIL (estimator drift too high)\n\n');
    end

    % Plot true vs estimated
    figure('Name', 'Sensor Failure: True vs Estimated', 'Position', [100 100 1000 600]);
    subplot(3,1,1);
    plot(data.time, data.position(:,1), 'b', data.time, data.est_pos(:,1), 'r--');
    ylabel('X [m]'); legend('True','Estimated'); title('Position: True vs Estimated');
    % Mark GPS outage
    xline(15, 'k--', 'GPS OFF'); xline(30, 'k--', 'GPS ON');
    grid on;

    subplot(3,1,2);
    plot(data.time, data.position(:,2), 'b', data.time, data.est_pos(:,2), 'r--');
    ylabel('Y [m]'); xline(15,'k--'); xline(30,'k--'); grid on;

    subplot(3,1,3);
    plot(data.time, -data.position(:,3), 'b', data.time, -data.est_pos(:,3), 'r--');
    ylabel('Alt [m]'); xlabel('Time [s]'); xline(15,'k--'); xline(30,'k--'); grid on;

end


function R = euler_to_dcm(phi, theta, psi)
    cphi=cos(phi); sphi=sin(phi);
    cth=cos(theta); sth=sin(theta);
    cpsi=cos(psi); spsi=sin(psi);
    R = [cth*cpsi, sphi*sth*cpsi-cphi*spsi, cphi*sth*cpsi+sphi*spsi;
         cth*spsi, sphi*sth*spsi+cphi*cpsi, cphi*sth*spsi-sphi*cpsi;
         -sth,     sphi*cth,                 cphi*cth                ];
end
