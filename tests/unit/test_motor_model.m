function results = test_motor_model()
% TEST_MOTOR_MODEL  Unit tests for motor_model_precise.m
%
%   results = test_motor_model()
%
%   Tests motor physics bounds, current limiting, efficiency curve shape,
%   voltage sag behavior, and pack current limiting.

    fprintf('\n========== MOTOR MODEL TESTS ==========\n');
    results.passed = 0;
    results.failed = 0;
    results.total  = 0;

    dp = drone_config('standard_quad');
    dp = dp.drone;
    n = dp.num_motors;

    %% Test 1: Zero command produces minimal thrust
    results = run_test(results, 'Zero command -> near-zero thrust', @() begin_test( ...
        @() test_zero_command(dp, n)));

    %% Test 2: Max command saturates at omega_max
    results = run_test(results, 'Max command saturates correctly', @() begin_test( ...
        @() test_max_saturation(dp, n)));

    %% Test 3: Motor speeds never go negative
    results = run_test(results, 'Motor speeds always non-negative', @() begin_test( ...
        @() test_non_negative(dp, n)));

    %% Test 4: Efficiency stays in [0.05, 1.0] range
    results = run_test(results, 'Efficiency in valid range [0.05, 1.0]', @() begin_test( ...
        @() test_efficiency_range(dp, n)));

    %% Test 5: Voltage sag increases with load
    results = run_test(results, 'Voltage sag increases with load', @() begin_test( ...
        @() test_voltage_sag(dp, n)));

    %% Test 6: Low SOC reduces ceiling speed
    results = run_test(results, 'Low SOC reduces max speed', @() begin_test( ...
        @() test_low_soc(dp, n)));

    %% Test 7: Pack current never exceeds max_current
    results = run_test(results, 'Pack current respects limit', @() begin_test( ...
        @() test_pack_current_limit(dp, n)));

    %% Summary
    fprintf('\n--- Motor Model: %d PASSED, %d FAILED (of %d) ---\n\n', ...
        results.passed, results.failed, results.total);
end

%% Individual test functions
function success = test_zero_command(dp, n)
    omega_cmd = zeros(n, 1);
    omega_cur = dp.omega_min * 2*pi/60 * ones(n, 1);
    [omega_out, thrust, ~, ~, ~, ~] = motor_model_precise(omega_cmd, omega_cur, 0.002, dp, 1.0);
    success = all(thrust >= 0) && all(thrust < 0.5);  % Near-zero thrust
end

function success = test_max_saturation(dp, n)
    omega_cmd = 1e6 * ones(n, 1);  % Way beyond max
    omega_cur = 500 * ones(n, 1);
    [omega_out, ~, ~, ~, ~, ~] = motor_model_precise(omega_cmd, omega_cur, 0.002, dp, 1.0);
    omega_max_rad = dp.Kv * 2*pi/60 * dp.V_full;
    success = all(omega_out <= omega_max_rad * 1.05);  % Within 5% of ceiling
end

function success = test_non_negative(dp, n)
    success = true;
    for soc = [1.0, 0.5, 0.2, 0.05]
        omega_cmd = rand(n, 1) * 1000;
        omega_cur = rand(n, 1) * 500;
        [omega_out, thrust, ~, current, ~, ~] = motor_model_precise(omega_cmd, omega_cur, 0.002, dp, soc);
        if any(omega_out < 0) || any(thrust < 0)
            success = false;
            return;
        end
    end
end

function success = test_efficiency_range(dp, n)
    success = true;
    for cmd_frac = 0.1:0.1:1.0
        omega_max = dp.Kv * 2*pi/60 * dp.V_nominal;
        omega_cmd = cmd_frac * omega_max * ones(n, 1);
        omega_cur = omega_cmd * 0.9;
        [~, ~, ~, ~, ~, eff] = motor_model_precise(omega_cmd, omega_cur, 0.002, dp, 0.8);
        if any(eff < 0.05) || any(eff > 1.0)
            success = false;
            return;
        end
    end
end

function success = test_voltage_sag(dp, n)
    % Compare voltage at low vs high load
    omega_low = 100 * ones(n, 1);
    omega_high = 800 * ones(n, 1);
    [~, ~, ~, ~, V_low, ~] = motor_model_precise(omega_low, omega_low, 0.002, dp, 0.8);
    [~, ~, ~, ~, V_high, ~] = motor_model_precise(omega_high, omega_high, 0.002, dp, 0.8);
    success = V_low > V_high;  % Higher load = more sag
end

function success = test_low_soc(dp, n)
    omega_cmd = 500 * ones(n, 1);
    omega_cur = 400 * ones(n, 1);
    [~, ~, ~, ~, V_full, ~] = motor_model_precise(omega_cmd, omega_cur, 0.002, dp, 1.0);
    [~, ~, ~, ~, V_low, ~]  = motor_model_precise(omega_cmd, omega_cur, 0.002, dp, 0.15);
    success = V_full > V_low;  % Low SOC = lower voltage
end

function success = test_pack_current_limit(dp, n)
    % Drive all motors at max command
    omega_cmd = 2000 * ones(n, 1);
    omega_cur = 1500 * ones(n, 1);
    [~, ~, ~, current, ~, ~] = motor_model_precise(omega_cmd, omega_cur, 0.002, dp, 0.5);
    I_total = sum(current);
    success = I_total <= dp.max_current * 1.01;  % 1% tolerance
end

%% Helper functions
function success = begin_test(fn)
    success = fn();
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
