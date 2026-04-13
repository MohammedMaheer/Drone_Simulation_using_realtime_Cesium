function post_flight_analysis(flight_data, save_plots)
% POST_FLIGHT_ANALYSIS  Generate comprehensive post-flight analysis plots.
%
%   post_flight_analysis(flight_data)
%   post_flight_analysis(flight_data, true)  % Also saves plots as PNG
%
%   Inputs:
%     flight_data - Struct from telemetry_logger.get_data() or loaded .mat
%     save_plots  - Boolean, save figures as PNG files (default: false)

    if nargin < 2; save_plots = false; end

    t   = flight_data.time;
    pos = flight_data.position;
    vel = flight_data.velocity;
    eul = rad2deg(flight_data.euler);
    omg = rad2deg(flight_data.omega);
    alt = -pos(:,3);  % NED to altitude

    fprintf('\n========== POST-FLIGHT ANALYSIS ==========\n');
    fprintf('Flight Duration:   %.1f s\n', t(end) - t(1));
    fprintf('Max Altitude:      %.2f m\n', max(alt));
    fprintf('Max Speed:         %.2f m/s\n', max(sqrt(sum(vel.^2, 2))));
    fprintf('Max Roll:          %.1f deg\n', max(abs(eul(:,1))));
    fprintf('Max Pitch:         %.1f deg\n', max(abs(eul(:,2))));
    fprintf('Final Battery:     %.1f%%\n', flight_data.battery_soc(end)*100);
    fprintf('Samples Logged:    %d\n', length(t));
    fprintf('===========================================\n\n');

    %% Figure 1: 3D Flight Path
    fig1 = figure('Name', 'Flight Path 3D', 'Position', [50 50 800 600]);
    plot3(pos(:,1), pos(:,2), alt, 'b-', 'LineWidth', 1.5);
    hold on;
    plot3(pos(1,1), pos(1,2), alt(1), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
    plot3(pos(end,1), pos(end,2), alt(end), 'rs', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
    grid on; xlabel('X North [m]'); ylabel('Y East [m]'); zlabel('Altitude [m]');
    title('3D Flight Path'); legend('Path', 'Start', 'End');
    view(30, 25);

    %% Figure 2: Position vs Time
    fig2 = figure('Name', 'Position', 'Position', [100 100 1000 600]);
    subplot(3,1,1); plot(t, pos(:,1), 'b'); grid on; ylabel('X [m]'); title('Position');
    subplot(3,1,2); plot(t, pos(:,2), 'r'); grid on; ylabel('Y [m]');
    subplot(3,1,3); plot(t, alt, 'g');      grid on; ylabel('Alt [m]'); xlabel('Time [s]');

    %% Figure 3: Velocity vs Time
    fig3 = figure('Name', 'Velocity', 'Position', [150 150 1000 600]);
    speed = sqrt(sum(vel.^2, 2));
    subplot(4,1,1); plot(t, vel(:,1), 'b'); grid on; ylabel('Vx [m/s]'); title('Velocity');
    subplot(4,1,2); plot(t, vel(:,2), 'r'); grid on; ylabel('Vy [m/s]');
    subplot(4,1,3); plot(t, vel(:,3), 'g'); grid on; ylabel('Vz [m/s]');
    subplot(4,1,4); plot(t, speed, 'k', 'LineWidth', 1.5); grid on;
    ylabel('Speed [m/s]'); xlabel('Time [s]');

    %% Figure 4: Attitude
    fig4 = figure('Name', 'Attitude', 'Position', [200 200 1000 600]);
    subplot(3,1,1); plot(t, eul(:,1), 'r'); grid on; ylabel('Roll [deg]'); title('Attitude');
    subplot(3,1,2); plot(t, eul(:,2), 'g'); grid on; ylabel('Pitch [deg]');
    subplot(3,1,3); plot(t, eul(:,3), 'b'); grid on; ylabel('Yaw [deg]'); xlabel('Time [s]');

    %% Figure 5: Control Effort
    fig5 = figure('Name', 'Control', 'Position', [250 250 1000 600]);
    subplot(2,1,1); plot(t, flight_data.thrust_cmd, 'k', 'LineWidth', 1.5);
    grid on; ylabel('Thrust [N]'); title('Control Commands');
    subplot(2,1,2); plot(t, flight_data.moment_cmds);
    grid on; ylabel('Moments [N·m]'); xlabel('Time [s]');
    legend('Mx (Roll)', 'My (Pitch)', 'Mz (Yaw)');

    %% Figure 6: Motor Speeds
    fig6 = figure('Name', 'Motors', 'Position', [300 300 1000 400]);
    rps_to_rpm = 60 / (2*pi);
    plot(t, flight_data.motor_speeds * rps_to_rpm);
    grid on; ylabel('Motor Speed [RPM]'); xlabel('Time [s]');
    title('Motor Speeds'); legend('M1','M2','M3','M4');

    %% Figure 7: Position Error
    if any(flight_data.pos_error(:) ~= 0)
        fig7 = figure('Name', 'Tracking Error', 'Position', [350 350 1000 400]);
        pos_err_norm = sqrt(sum(flight_data.pos_error.^2, 2));
        plot(t, pos_err_norm, 'r', 'LineWidth', 1.5);
        grid on; ylabel('Position Error [m]'); xlabel('Time [s]');
        title(sprintf('Position Tracking Error (RMS = %.3f m)', rms(pos_err_norm)));
    end

    %% Figure 8: Battery SOC
    fig8 = figure('Name', 'Battery', 'Position', [400 400 800 300]);
    plot(t, flight_data.battery_soc * 100, 'LineWidth', 2);
    grid on; ylabel('SOC [%]'); xlabel('Time [s]'); title('Battery State of Charge');
    ylim([0 105]);

    %% Save plots
    if save_plots
        saveas(fig1, 'plot_flight_path_3d.png');
        saveas(fig2, 'plot_position.png');
        saveas(fig3, 'plot_velocity.png');
        saveas(fig4, 'plot_attitude.png');
        saveas(fig5, 'plot_control.png');
        saveas(fig6, 'plot_motors.png');
        saveas(fig8, 'plot_battery.png');
        fprintf('Plots saved to current directory.\n');
    end

end
