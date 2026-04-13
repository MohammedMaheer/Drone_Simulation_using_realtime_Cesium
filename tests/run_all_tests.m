function run_all_tests()
% RUN_ALL_TESTS  Execute all unit and integration tests.
%
%   run_all_tests()
%
%   Discovers and runs all test_*.m files in tests/unit/ and tests/integration/,
%   then prints a comprehensive summary.

    fprintf('\n');
    fprintf('╔══════════════════════════════════════════════════════╗\n');
    fprintf('║        DRONE SIMULATION — TEST SUITE                ║\n');
    fprintf('╚══════════════════════════════════════════════════════╝\n');
    fprintf('  Time: %s\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
    fprintf('\n');

    % Add all paths
    root = fileparts(mfilename('fullpath'));
    addpath(genpath(root));

    total_passed = 0;
    total_failed = 0;
    total_tests  = 0;
    suite_results = {};

    %% Unit Tests
    fprintf('━━━━━━━━━━ UNIT TESTS ━━━━━━━━━━\n');
    suites = {'test_config', 'test_motor_model', 'test_dynamics', 'test_wind_model'};
    for i = 1:length(suites)
        try
            r = feval(suites{i});
            total_passed = total_passed + r.passed;
            total_failed = total_failed + r.failed;
            total_tests  = total_tests + r.total;
            suite_results{end+1} = struct('name', suites{i}, 'passed', r.passed, ...
                'failed', r.failed, 'total', r.total);
        catch ME
            fprintf('  [SUITE ERROR] %s: %s\n', suites{i}, ME.message);
            total_failed = total_failed + 1;
            total_tests = total_tests + 1;
            suite_results{end+1} = struct('name', suites{i}, 'passed', 0, ...
                'failed', 1, 'total', 1);
        end
    end

    %% Integration Tests
    fprintf('━━━━━━━━━━ INTEGRATION TESTS ━━━━━━━━━━\n');
    int_suites = {'test_hover_stability'};
    for i = 1:length(int_suites)
        try
            r = feval(int_suites{i});
            total_passed = total_passed + r.passed;
            total_failed = total_failed + r.failed;
            total_tests  = total_tests + r.total;
            suite_results{end+1} = struct('name', int_suites{i}, 'passed', r.passed, ...
                'failed', r.failed, 'total', r.total);
        catch ME
            fprintf('  [SUITE ERROR] %s: %s\n', int_suites{i}, ME.message);
            total_failed = total_failed + 1;
            total_tests = total_tests + 1;
            suite_results{end+1} = struct('name', int_suites{i}, 'passed', 0, ...
                'failed', 1, 'total', 1);
        end
    end

    %% Final Summary
    fprintf('\n');
    fprintf('╔══════════════════════════════════════════════════════╗\n');
    fprintf('║  FINAL RESULTS                                      ║\n');
    fprintf('╠══════════════════════════════════════════════════════╣\n');
    for i = 1:length(suite_results)
        s = suite_results{i};
        if s.failed == 0
            status = '✓';
        else
            status = '✗';
        end
        fprintf('║  %s %-30s  %2d/%2d passed     ║\n', status, s.name, s.passed, s.total);
    end
    fprintf('╠══════════════════════════════════════════════════════╣\n');
    if total_failed == 0
        fprintf('║  ✓ ALL TESTS PASSED: %d/%d                          ║\n', total_passed, total_tests);
    else
        fprintf('║  ✗ %d PASSED, %d FAILED (of %d total)               ║\n', ...
            total_passed, total_failed, total_tests);
    end
    fprintf('╚══════════════════════════════════════════════════════╝\n\n');

end
