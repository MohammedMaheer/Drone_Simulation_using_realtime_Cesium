function test_wind_disturbance()
% TEST_WIND_DISTURBANCE  Test position hold under wind gusts.
%
%   Verifies the controller can reject wind disturbances while hovering.

    fprintf('=== TEST: Wind Disturbance Rejection ===\n');

    %% Initialize
    dp = drone_params();
    cp = controller_params();
    sim_p = sim_params();

    clear attitude_controller position_controller altitude_controller
    clear flight_controller waypoint_manager

    %% Target
    target.position = [0; 0; -10];
    target.yaw = 0;

    %% Simulation
    dt = sim_p.dt;
    t_end = 40;
    N = round(t_end / dt);

    state = zeros(12, 1);
    state(3) = -10;  % Start at hover altitude
    motor_speeds = ones(4,1) * dp.hover_omega;

    logger = telemetry_logger(N);
    batt_soc = 1.0;

    fprintf('Running wind disturbance test for %.0f seconds...\n', t_end);
    fprintf('Wind profile: calm → gust → calm → gust\n');

    for i = 1:N
        t = (i-1) * dt;

        % Wind profile: gusts at 10-15s and 25-30s
        wind = [0; 0; 0];
        if t >= 10 && t < 15
            wind = [5; 3; -1];  % Strong gust
        elseif t >= 25 && t < 30
            wind = [-4; 6; 0.5]; % Different direction
        end

        % Add turbulence
        wind = wind + 0.5 * randn(3,1);

        % Controller
        [thrust_cmd, moment_cmds, ctrl_data] = flight_controller(state, target, dp, cp);
        motor_cmds = mixing_matrix(thrust_cmd, moment_cmds, dp);
        [motor_speeds, ~, ~, current] = motor_model(motor_cmds, motor_speeds, dt, dp);

        % Plant
        [state_dot, ~, ~] = quadrotor_dynamics(state, motor_speeds, wind, dp);
        state = state + state_dot * dt;

        batt_soc = batt_soc - sum(current) * dt / (dp.capacity_Ah * 3600);

        if mod(i, 10) == 0
            logger.log(t, state, motor_speeds, ctrl_data, [], [], batt_soc, [], wind);
        end
    end

    %% Results
    data = logger.get_data();

    % Compute max displacement during gusts
    gust1_idx = data.time >= 10 & data.time < 20;
    gust2_idx = data.time >= 25 & data.time < 35;

    max_disp1 = max(sqrt(sum(data.position(gust1_idx, 1:2).^2, 2)));
    max_disp2 = max(sqrt(sum(data.position(gust2_idx, 1:2).^2, 2)));
    max_alt_err = max(abs(-data.position(:,3) - 10));

    fprintf('\n--- Wind Disturbance Results ---\n');
    fprintf('Max XY displacement (gust 1): %.2f m\n', max_disp1);
    fprintf('Max XY displacement (gust 2): %.2f m\n', max_disp2);
    fprintf('Max altitude error:           %.2f m\n', max_alt_err);

    recovery_pos = norm(data.position(end, 1:2));
    fprintf('Final position drift:         %.3f m\n', recovery_pos);

    if max_disp1 < 8 && max_disp2 < 8 && recovery_pos < 2
        fprintf('RESULT: PASS\n\n');
    else
        fprintf('RESULT: FAIL\n\n');
    end

    post_flight_analysis(data);

end
