function results = test_config()
% TEST_CONFIG  Unit tests for drone_config.m
%
%   results = test_config()
%
%   Tests all 5 presets produce physically valid configurations,
%   and verifies auto-derivation correctness.

    fprintf('\n========== CONFIG TESTS ==========\n');
    results.passed = 0;
    results.failed = 0;
    results.total  = 0;

    presets = {'mini_quad', 'standard_quad', 'heavy_hex', 'octo_lift', 'micro_tri'};

    %% Test each preset
    for i = 1:length(presets)
        name = presets{i};
        results = run_test(results, sprintf('%s loads without error', name), ...
            @() test_preset_loads(name));
        results = run_test(results, sprintf('%s has valid T/W ratio', name), ...
            @() test_tw_ratio(name));
        results = run_test(results, sprintf('%s has positive inertia', name), ...
            @() test_inertia(name));
        results = run_test(results, sprintf('%s has matching motor count', name), ...
            @() test_motor_count(name));
        results = run_test(results, sprintf('%s has valid battery', name), ...
            @() test_battery(name));
    end

    %% Test auto-derivation consistency
    results = run_test(results, 'kT and hover_omega consistent', @test_kt_consistency);
    results = run_test(results, 'Motor layout positions match arm_length', @test_layout_positions);

    %% Summary
    fprintf('\n--- Config: %d PASSED, %d FAILED (of %d) ---\n\n', ...
        results.passed, results.failed, results.total);
end

function success = test_preset_loads(preset)
    try
        cfg = drone_config(preset);
        success = isstruct(cfg) && isfield(cfg, 'drone') && isfield(cfg, 'controller');
    catch
        success = false;
    end
end

function success = test_tw_ratio(preset)
    cfg = drone_config(preset);
    dp = cfg.drone;
    % Thrust-to-weight should be between 1.5 and 15
    % Use omega_max (85% of no-load) which is the actual operating limit
    max_omega = dp.omega_max * 2*pi/60;
    max_thrust = dp.num_motors * dp.kT * max_omega^2;
    tw_ratio = max_thrust / (dp.mass * dp.g);
    success = tw_ratio > 1.5 && tw_ratio < 15;
end

function success = test_inertia(preset)
    cfg = drone_config(preset);
    dp = cfg.drone;
    I = dp.I;
    success = all(diag(I) > 0) && issymmetric(I);
end

function success = test_motor_count(preset)
    cfg = drone_config(preset);
    dp = cfg.drone;
    n = dp.num_motors;
    success = length(dp.motor_layout.spin_dirs) == n && ...
              size(dp.motor_layout.positions, 1) == n;
end

function success = test_battery(preset)
    cfg = drone_config(preset);
    dp = cfg.drone;
    success = dp.V_full > dp.V_nominal && dp.V_nominal > dp.V_empty && ...
              dp.energy_Wh > 0 && dp.max_current > 0 && dp.R_internal > 0;
end

function success = test_kt_consistency()
    cfg = drone_config('standard_quad');
    dp = cfg.drone;
    % At hover, total thrust = weight
    hover_omega = sqrt(dp.mass * dp.g / (dp.num_motors * dp.kT));
    % This should be within reasonable RPM range
    hover_rpm = hover_omega * 60 / (2*pi);
    max_rpm = dp.Kv * dp.V_nominal;
    hover_fraction = hover_rpm / max_rpm;
    success = hover_fraction > 0.2 && hover_fraction < 0.85;  % 20-85% throttle at hover
end

function success = test_layout_positions()
    cfg = drone_config('standard_quad');
    dp = cfg.drone;
    positions = dp.motor_layout.positions;
    % All motor positions should be approximately arm_length from center
    distances = sqrt(sum(positions.^2, 2));
    success = all(abs(distances - dp.arm_length) < 0.01);
end

function success = issymmetric(M)
    success = norm(M - M', 'fro') < 1e-10;
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
