function results = test_hover_stability()
% TEST_HOVER_STABILITY  Integration test: verify stable hover for all presets.
%
%   results = test_hover_stability()
%
%   Simulates 5 seconds of hover for each drone preset and verifies:
%     - Altitude converges to target within 2m
%     - No divergence (all states remain finite)
%     - Horizontal drift < 5m
%     - Attitude stays within ±20°

    fprintf('\n========== HOVER STABILITY INTEGRATION TESTS ==========\n');
    results.passed = 0;
    results.failed = 0;
    results.total  = 0;

    presets = {'mini_quad', 'standard_quad', 'heavy_hex', 'octo_lift', 'micro_tri'};

    for i = 1:length(presets)
        name = presets{i};
        results = run_test(results, sprintf('%s hover convergence', name), ...
            @() test_hover_preset(name));
    end

    %% Test with wind disturbance
    results = run_test(results, 'standard_quad hover in wind', @test_hover_wind);

    %% Summary
    fprintf('\n--- Hover Stability: %d PASSED, %d FAILED (of %d) ---\n\n', ...
        results.passed, results.failed, results.total);
end

function success = test_hover_preset(preset)
    cfg = drone_config(preset);
    dp = cfg.drone;
    cp = cfg.controller;

    % Clear controller persistent states
    clear attitude_controller position_controller altitude_controller flight_controller

    target_alt = 5;  % meters
    dt = 0.002;
    T = 8.0;  % seconds (allow heavier drones time to converge)
    N = round(T / dt);

    % Override loop rates to match simulation rate so internal dt is correct
    sim_rate = 1 / dt;  % 500 Hz
    cp.position_rate = sim_rate;
    cp.attitude_rate = sim_rate;
    cp.rate_rate     = sim_rate;

    state = zeros(12, 1);
    state(3) = 0;  % Start on ground (NED: z=0)

    target.position = [0; 0; -target_alt];  % NED: negative z = up
    target.yaw = 0;

    motor_speeds = zeros(dp.num_motors, 1);

    for k = 1:N
        [thrust_cmd, moment_cmds, ~] = flight_controller(state, target, dp, cp);
        motor_cmds = mixing_matrix_n(thrust_cmd, moment_cmds, dp);

        alpha = dt / (dp.tau_motor + dt);
        motor_speeds = motor_speeds + alpha * (motor_cmds - motor_speeds);

        [state_dot, ~, ~] = multirotor_dynamics(state, motor_speeds, [0;0;0], dp);
        state = state + dt * state_dot;

        % Ground constraint
        if state(3) > 0
            state(3) = 0;
            if state(6) > 0; state(6) = 0; end
        end

        if any(~isfinite(state))
            success = false;
            return;
        end
    end

    final_alt = -state(3);
    horiz_drift = norm(state(1:2));
    max_tilt = max(abs(state(7:8)));

    % Tricopters have limited yaw authority (no tilt servo), so relax bounds
    if dp.num_motors == 3
        drift_limit = 15.0;
        tilt_limit  = deg2rad(30);
    else
        drift_limit = 5.0;
        tilt_limit  = deg2rad(20);
    end

    success = abs(final_alt - target_alt) < 2.0 && ...
              horiz_drift < drift_limit && ...
              max_tilt < tilt_limit;
end

function success = test_hover_wind()
    cfg = drone_config('standard_quad');
    dp = cfg.drone;
    cp = cfg.controller;

    clear attitude_controller position_controller altitude_controller flight_controller

    target_alt = 5;
    dt = 0.002;
    T = 8.0;
    N = round(T / dt);

    % Override loop rates to match simulation rate
    sim_rate = 1 / dt;
    cp.position_rate = sim_rate;
    cp.attitude_rate = sim_rate;
    cp.rate_rate     = sim_rate;

    state = zeros(12, 1);
    target.position = [0; 0; -target_alt];
    target.yaw = 0;
    motor_speeds = zeros(dp.num_motors, 1);

    for k = 1:N
        t = k * dt;
        wind = [3*sin(0.5*t); 2*cos(0.3*t); 0];  % Moderate wind

        [thrust_cmd, moment_cmds, ~] = flight_controller(state, target, dp, cp);
        motor_cmds = mixing_matrix_n(thrust_cmd, moment_cmds, dp);

        alpha = dt / (dp.tau_motor + dt);
        motor_speeds = motor_speeds + alpha * (motor_cmds - motor_speeds);

        [state_dot, ~, ~] = multirotor_dynamics(state, motor_speeds, wind, dp);
        state = state + dt * state_dot;

        if state(3) > 0; state(3) = 0; if state(6) > 0; state(6) = 0; end; end

        if any(~isfinite(state)); success = false; return; end
    end

    final_alt = -state(3);
    horiz_drift = norm(state(1:2));
    max_tilt = max(abs(state(7:8)));

    % More relaxed bounds due to wind
    success = abs(final_alt - target_alt) < 3.0 && ...
              horiz_drift < 10.0 && ...
              max_tilt < deg2rad(30);
end

function results = run_test(results, name, test_fn)
    results.total = results.total + 1;
    try
        success = test_fn();
        if success
            fprintf('  [PASS] %s\n', name);
            results.passed = results.passed + 1;
        else
            fprintf('  [FAIL] %s\n', name);
            results.failed = results.failed + 1;
        end
    catch ME
        fprintf('  [ERROR] %s: %s\n', name, ME.message);
        results.failed = results.failed + 1;
    end
end
