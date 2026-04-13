function results = test_dynamics()
% TEST_DYNAMICS  Unit tests for multirotor_dynamics.m and quadrotor_dynamics.m
%
%   results = test_dynamics()
%
%   Tests physics correctness: hover equilibrium, gravity, ground contact,
%   gimbal lock protection, and conservation properties.

    fprintf('\n========== DYNAMICS TESTS ==========\n');
    results.passed = 0;
    results.failed = 0;
    results.total  = 0;

    cfg = drone_config('standard_quad');
    dp = cfg.drone;

    %% Test 1: Hover thrust balances gravity
    results = run_test(results, 'Hover thrust balances gravity', @() test_hover(dp));

    %% Test 2: Free-fall (zero motors) gives g acceleration downward
    results = run_test(results, 'Free-fall acceleration = g', @() test_freefall(dp));

    %% Test 3: Ground contact prevents penetration
    results = run_test(results, 'Ground contact prevents z > 0', @() test_ground(dp));

    %% Test 4: Gimbal lock protection at extreme pitch
    results = run_test(results, 'Gimbal lock protection works', @() test_gimbal_lock(dp));

    %% Test 5: Yaw torque from differential motor speed
    results = run_test(results, 'Differential speed produces yaw', @() test_yaw_torque(dp));

    %% Test 6: Drag opposes velocity
    results = run_test(results, 'Drag force opposes velocity', @() test_drag(dp));

    %% Test 7: Ground effect increases thrust near ground
    results = run_test(results, 'Ground effect boosts thrust near ground', @() test_ground_effect(dp));

    %% Summary
    fprintf('\n--- Dynamics: %d PASSED, %d FAILED (of %d) ---\n\n', ...
        results.passed, results.failed, results.total);
end

function success = test_hover(dp)
    % At hover, vertical force should be approximately zero
    hover_omega = sqrt(dp.mass * dp.g / (dp.num_motors * dp.kT));
    state = zeros(12, 1);
    state(3) = -5;  % 5m altitude (NED: negative = up)
    motor_speeds = hover_omega * ones(dp.num_motors, 1);
    wind = [0; 0; 0];
    [state_dot, forces, ~] = multirotor_dynamics(state, motor_speeds, wind, dp);
    % Vertical acceleration should be near zero
    az = forces(3) / dp.mass;
    success = abs(az) < 0.1;  % Within 0.1 m/s^2
end

function success = test_freefall(dp)
    state = zeros(12, 1);
    state(3) = -10;  % 10m altitude
    motor_speeds = zeros(dp.num_motors, 1);
    wind = [0; 0; 0];
    [state_dot, forces, ~] = multirotor_dynamics(state, motor_speeds, wind, dp);
    az = forces(3) / dp.mass;
    success = abs(az - dp.g) < 0.01;  % Should be g (downward in NED)
end

function success = test_ground(dp)
    state = zeros(12, 1);
    state(3) = 0;  % On the ground
    state(6) = 1;  % Moving downward
    motor_speeds = zeros(dp.num_motors, 1);
    wind = [0; 0; 0];
    [state_dot, forces, ~] = multirotor_dynamics(state, motor_speeds, wind, dp);
    success = forces(3) <= 0;  % No downward force through ground
end

function success = test_gimbal_lock(dp)
    state = zeros(12, 1);
    state(3) = -5;
    state(8) = deg2rad(89);  % Near gimbal lock pitch
    state(11) = 1.0;  % Some pitch rate
    motor_speeds = 500 * ones(dp.num_motors, 1);
    wind = [0; 0; 0];
    try
        [state_dot, ~, ~] = multirotor_dynamics(state, motor_speeds, wind, dp);
        success = all(isfinite(state_dot));  % No Inf or NaN
    catch
        success = false;
    end
end

function success = test_yaw_torque(dp)
    state = zeros(12, 1);
    state(3) = -5;
    % Set motors to create yaw torque (all spinning faster one direction)
    n = dp.num_motors;
    motor_speeds = 500 * ones(n, 1);
    % Increase CW motors, decrease CCW motors
    for i = 1:n
        if dp.motor_layout.spin_dirs(i) > 0  % CCW
            motor_speeds(i) = 600;
        else
            motor_speeds(i) = 400;
        end
    end
    wind = [0; 0; 0];
    [~, ~, moments] = multirotor_dynamics(state, motor_speeds, wind, dp);
    success = abs(moments(3)) > 0.001;  % Non-zero yaw torque
end

function success = test_drag(dp)
    state = zeros(12, 1);
    state(3) = -5;
    state(4) = 10;  % Flying forward at 10 m/s
    motor_speeds = 500 * ones(dp.num_motors, 1);
    wind = [0; 0; 0];
    [~, forces, ~] = multirotor_dynamics(state, motor_speeds, wind, dp);
    % Drag should push backward (negative x force component)
    % Actually, the drag component depends on total forces. Let's check
    % that forces in x are negative of velocity direction (at least partially)
    % We need to remove gravity and thrust to isolate drag
    state2 = state;
    state2(4) = 0;  % No velocity
    [~, forces0, ~] = multirotor_dynamics(state2, motor_speeds, wind, dp);
    drag_force_x = forces(1) - forces0(1);
    success = drag_force_x < 0;  % Drag opposes positive x velocity
end

function success = test_ground_effect(dp)
    state_high = zeros(12, 1);
    state_high(3) = -5;  % 5m (well above ground effect)
    state_low = zeros(12, 1);
    state_low(3) = -0.3;  % 0.3m (in ground effect zone)
    motor_speeds = 500 * ones(dp.num_motors, 1);
    wind = [0; 0; 0];
    [~, forces_high, ~] = multirotor_dynamics(state_high, motor_speeds, wind, dp);
    [~, forces_low, ~] = multirotor_dynamics(state_low, motor_speeds, wind, dp);
    % Near-ground thrust should be greater (more negative z force)
    success = forces_low(3) < forces_high(3);  % More upward force near ground
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
