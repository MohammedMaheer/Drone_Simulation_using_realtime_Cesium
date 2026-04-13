function flight_data = run_simulation(mission_name, options)
% RUN_SIMULATION  Main entry point for running the drone simulation.
%
%   run_simulation()                             — Default hover mission
%   run_simulation('circle')                     — Circle mission
%   run_simulation('square', 'Wind', true)       — Square with wind
%   run_simulation('figure8', 'Duration', 90)    — Figure-8 for 90s
%   data = run_simulation(...)                    — Return flight data
%
%   Options (name-value pairs):
%     'Duration'    - Simulation duration [s] (default: 60)
%     'Wind'        - Enable wind (default: false)
%     'Sensors'     - Enable sensor noise (default: false)
%     'Dashboard'   - Show live dashboard (default: true)
%     'Animate'     - Play animation after sim (default: false)
%     'SaveLog'     - Save flight log to file (default: false)

    arguments
        mission_name (1,:) char = 'hover'
        options.Duration (1,1) double = 60
        options.Wind (1,1) logical = false
        options.Sensors (1,1) logical = false
        options.Dashboard (1,1) logical = true
        options.Animate (1,1) logical = false
        options.SaveLog (1,1) logical = false
    end

    %% Initialize parameters
    dp = drone_params();
    cp = controller_params();
    sp = sensor_params();
    sim_p = sim_params();

    % Clear persistent variables in all controllers/sensors
    clear attitude_controller position_controller altitude_controller
    clear flight_controller waypoint_manager imu_model gps_model
    clear barometer_model magnetometer_model state_estimator

    %% Setup mission
    mission_params.altitude = sim_p.scenario.target_alt;
    mission_params.size = 20;
    waypoints = mission_profiles(mission_name, mission_params);

    wp_params.accept_radius = 2.0;
    wp_params.accept_alt    = 1.5;
    wp_params.loiter_time   = 2.0;

    fprintf('\n============================\n');
    fprintf(' DRONE SIMULATION\n');
    fprintf('============================\n');
    fprintf('Mission:    %s\n', mission_name);
    fprintf('Waypoints:  %d\n', size(waypoints, 1));
    fprintf('Duration:   %.0f s\n', options.Duration);
    fprintf('Wind:       %s\n', ternary(options.Wind, 'ON', 'OFF'));
    fprintf('Sensors:    %s\n', ternary(options.Sensors, 'ON (noisy)', 'OFF (perfect)'));
    fprintf('============================\n\n');

    %% Simulation configuration
    dt = sim_p.dt;
    t_end = options.Duration;
    N = round(t_end / dt);
    log_interval = round(sim_p.dt_log / dt);  % Log every 10ms
    dash_interval = round(sim_p.dt_viz / dt);  % Dashboard every 20ms

    %% Initial state
    state = [sim_p.init.position; sim_p.init.velocity; ...
             sim_p.init.euler; sim_p.init.omega];
    motor_speeds = sim_p.init.motor_speed;
    if all(motor_speeds == 0)
        motor_speeds = ones(4,1) * 100;  % Small initial spin
    end

    %% Setup logging
    logger = telemetry_logger(ceil(N / log_interval) + 100);
    batt_soc = 1.0;

    %% Wind model
    wind_mean = sim_p.env.wind_mean;
    wind_gust_amp = sim_p.env.wind_gust_amp;
    wind_gust_freq = sim_p.env.wind_gust_freq;

    %% Main simulation loop
    fprintf('Simulating');
    tic;

    for i = 1:N
        t = (i-1) * dt;

        %% Wind
        if options.Wind
            wind_gust = wind_gust_amp .* sin(2*pi*wind_gust_freq*t);
            wind_turb = 0.3 * randn(3,1);
            wind = wind_mean + wind_gust + wind_turb;
        else
            wind = [0; 0; 0];
        end

        %% Navigation — get current target
        [target, wp_status] = waypoint_manager(state(1:3), waypoints, wp_params);

        %% Flight Controller
        [thrust_cmd, moment_cmds, ctrl_data] = flight_controller(state, target, dp, cp);

        %% Control Allocation
        motor_cmds = mixing_matrix(thrust_cmd, moment_cmds, dp);

        %% Motor Dynamics
        [motor_speeds, ~, ~, current] = motor_model(motor_cmds, motor_speeds, dt, dp);

        %% Sensors (optional)
        sensor_data = [];
        est_state = [];
        if options.Sensors
            R = euler_to_dcm_local(state(7), state(8), state(9));
            accel_true = R' * ([0;0;dp.g] + [0;0;0]);  % Specific force
            [accel_m, gyro_m] = imu_model(accel_true, state(10:12), dt, sp);
            [gps_pos, gps_vel, gps_valid] = gps_model(state(1:3), state(4:6), t, sp);
            baro_alt = barometer_model(-state(3), t, sp);
            mag_head = magnetometer_model(state(7:9), sp);

            sensor_data.accel = accel_m;
            sensor_data.gyro = gyro_m;
            sensor_data.gps_pos = gps_pos;
            sensor_data.gps_vel = gps_vel;
            sensor_data.gps_valid = gps_valid;
            sensor_data.baro_alt = baro_alt;
            sensor_data.mag_heading = mag_head;

            [est_state, ~] = state_estimator(sensor_data, dt, sp, dp);
        end

        %% Plant Dynamics
        [state_dot, ~, ~] = quadrotor_dynamics(state, motor_speeds, wind, dp);
        state = state + state_dot * dt;

        %% Battery Model
        total_current = sum(current);
        batt_soc = max(0, batt_soc - total_current * dt / (dp.capacity_Ah * 3600));

        %% Telemetry Logging
        if mod(i, log_interval) == 0
            logger.log(t, state, motor_speeds, ctrl_data, sensor_data, est_state, ...
                       batt_soc, wp_status, wind);
        end

        %% Live Dashboard
        if options.Dashboard && mod(i, dash_interval) == 0
            telemetry_dashboard(state, ctrl_data, batt_soc, wp_status, t);
        end

        %% Progress indicator
        if mod(i, N/10) < 1
            fprintf('.');
        end

        %% Safety checks
        alt = -state(3);
        if alt < -5
            fprintf('\nWARNING: Drone crashed (altitude = %.1f m)\n', alt);
            break;
        end
        if batt_soc <= 0
            fprintf('\nWARNING: Battery depleted at t=%.1f s\n', t);
            break;
        end
    end

    elapsed = toc;
    fprintf(' Done! (%.1f s real time for %.0f s sim)\n\n', elapsed, t_end);

    %% Get flight data
    flight_data = logger.get_data();

    %% Post-flight summary
    fprintf('--- Flight Summary ---\n');
    fprintf('Max Altitude:   %.1f m\n', max(-flight_data.position(:,3)));
    fprintf('Max Speed:      %.1f m/s\n', max(sqrt(sum(flight_data.velocity.^2, 2))));
    fprintf('Final Battery:  %.1f%%\n', batt_soc * 100);
    fprintf('Waypoints:      %d/%d completed\n', wp_status.current_idx, wp_status.total);
    fprintf('Mission Status: %s\n\n', ternary(wp_status.completed, 'COMPLETE', 'IN PROGRESS'));

    %% Save log
    if options.SaveLog
        logger.save_log(sprintf('flight_log_%s_%s.mat', mission_name, ...
            datestr(now, 'yyyymmdd_HHMMSS')));
    end

    %% Post-flight plots
    plot_flight_data(flight_data);

    %% Animation
    if options.Animate
        animate_flight(flight_data, 2.0);
    end

end


function result = ternary(cond, a, b)
    if cond; result = a; else; result = b; end
end

function R = euler_to_dcm_local(phi, theta, psi)
    cphi=cos(phi); sphi=sin(phi);
    cth=cos(theta); sth=sin(theta);
    cpsi=cos(psi); spsi=sin(psi);
    R = [cth*cpsi, sphi*sth*cpsi-cphi*spsi, cphi*sth*cpsi+sphi*spsi;
         cth*spsi, sphi*sth*spsi+cphi*cpsi, cphi*sth*spsi-sphi*cpsi;
         -sth,     sphi*cth,                 cphi*cth                ];
end
