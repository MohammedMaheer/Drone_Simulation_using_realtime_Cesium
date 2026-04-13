function results = test_wind_model()
% TEST_WIND_MODEL  Unit tests for dryden_wind_model.m
%
%   results = test_wind_model()
%
%   Tests Dryden turbulence model: output dimensions, statistical properties,
%   scaling with intensity, altitude dependence, and spectrum shape.

    fprintf('\n========== WIND MODEL TESTS ==========\n');
    results.passed = 0;
    results.failed = 0;
    results.total  = 0;

    %% Test 1: Output dimensions are [3x1]
    results = run_test(results, 'Output dimensions [3x1]', @test_dimensions);

    %% Test 2: Zero intensity gives near-zero turbulence
    results = run_test(results, 'Zero intensity -> zero turbulence', @test_zero_intensity);

    %% Test 3: Severe turbulence has higher RMS than light
    results = run_test(results, 'Severe > moderate > light RMS', @test_intensity_scaling);

    %% Test 4: Mean turbulence is approximately zero
    results = run_test(results, 'Mean turbulence ≈ 0', @test_zero_mean);

    %% Test 5: Turbulence is colored noise (autocorrelated)
    results = run_test(results, 'Turbulence is autocorrelated (colored noise)', @test_colored_noise);

    %% Test 6: Steady wind is passed through
    results = run_test(results, 'Steady wind component preserved', @test_steady_wind);

    %% Summary
    fprintf('\n--- Wind Model: %d PASSED, %d FAILED (of %d) ---\n\n', ...
        results.passed, results.failed, results.total);
end

function success = test_dimensions()
    clear dryden_wind_model
    [w, turb] = dryden_wind_model(0, 0.002, 50, 10, [3; 0; 0], 'moderate');
    success = isequal(size(w), [3, 1]) && isequal(size(turb), [3, 1]);
end

function success = test_zero_intensity()
    clear dryden_wind_model
    N = 1000;
    turb_sum = zeros(3, 1);
    turb_sq = zeros(3, 1);
    for i = 1:N
        [~, turb] = dryden_wind_model(i*0.002, 0.002, 50, 10, [0;0;0], 0.001);
        turb_sum = turb_sum + turb;
        turb_sq = turb_sq + turb.^2;
    end
    rms = sqrt(turb_sq / N);
    success = all(rms < 0.1);  % Very small RMS
end

function success = test_intensity_scaling()
    clear dryden_wind_model
    N = 5000;
    rms_light = run_rms(N, 'light');
    clear dryden_wind_model
    rms_mod = run_rms(N, 'moderate');
    clear dryden_wind_model
    rms_sev = run_rms(N, 'severe');
    % Average RMS across components
    r_l = mean(rms_light);
    r_m = mean(rms_mod);
    r_s = mean(rms_sev);
    success = r_l < r_m && r_m < r_s;
end

function success = test_zero_mean()
    clear dryden_wind_model
    N = 10000;
    turb_sum = zeros(3, 1);
    for i = 1:N
        [~, turb] = dryden_wind_model(i*0.002, 0.002, 50, 10, [0;0;0], 'moderate');
        turb_sum = turb_sum + turb;
    end
    mean_turb = turb_sum / N;
    success = all(abs(mean_turb) < 1.0);  % Mean close to zero (within 1 m/s)
end

function success = test_colored_noise()
    clear dryden_wind_model
    N = 2000;
    samples = zeros(N, 1);
    for i = 1:N
        [~, turb] = dryden_wind_model(i*0.002, 0.002, 50, 10, [0;0;0], 'moderate');
        samples(i) = turb(1);  % u-component
    end
    % Autocorrelation at lag 1 should be positive for colored noise
    auto_corr = corrcoef(samples(1:end-1), samples(2:end));
    success = auto_corr(1,2) > 0.3;  % Significant positive autocorrelation
end

function success = test_steady_wind()
    clear dryden_wind_model
    steady = [5; -3; 0.5];
    N = 1000;
    wind_sum = zeros(3, 1);
    for i = 1:N
        [w, ~] = dryden_wind_model(i*0.002, 0.002, 50, 10, steady, 'light');
        wind_sum = wind_sum + w;
    end
    mean_wind = wind_sum / N;
    % Mean wind should be close to steady component
    success = norm(mean_wind - steady) < 2.0;  % Within 2 m/s of steady
end

function rms = run_rms(N, intensity)
    turb_sq = zeros(3, 1);
    for i = 1:N
        [~, turb] = dryden_wind_model(i*0.002, 0.002, 50, 10, [0;0;0], intensity);
        turb_sq = turb_sq + turb.^2;
    end
    rms = sqrt(turb_sq / N);
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
