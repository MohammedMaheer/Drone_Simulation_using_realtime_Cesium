function plot_flight_data(flight_data)
% PLOT_FLIGHT_DATA  Quick overview plots of flight telemetry.
%
%   plot_flight_data(flight_data)
%
%   Creates a single figure with 6 subplots showing key flight parameters.

    t   = flight_data.time;
    pos = flight_data.position;
    vel = flight_data.velocity;
    eul = rad2deg(flight_data.euler);
    alt = -pos(:,3);

    figure('Name', 'Flight Data Overview', 'Position', [100 100 1200 800]);

    % Position XY
    subplot(2,3,1);
    plot(pos(:,1), pos(:,2), 'b-', 'LineWidth', 1.2);
    hold on;
    plot(pos(1,1), pos(1,2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    plot(pos(end,1), pos(end,2), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    grid on; axis equal; xlabel('X [m]'); ylabel('Y [m]');
    title('Ground Track'); legend('Path','Start','End');

    % Altitude
    subplot(2,3,2);
    plot(t, alt, 'b', 'LineWidth', 1.2);
    grid on; xlabel('Time [s]'); ylabel('Altitude [m]');
    title('Altitude');

    % Speed
    subplot(2,3,3);
    speed = sqrt(sum(vel.^2, 2));
    plot(t, speed, 'r', 'LineWidth', 1.2);
    grid on; xlabel('Time [s]'); ylabel('Speed [m/s]');
    title('Speed');

    % Attitude
    subplot(2,3,4);
    plot(t, eul(:,1), 'r', t, eul(:,2), 'g', t, eul(:,3), 'b');
    grid on; xlabel('Time [s]'); ylabel('Angle [deg]');
    title('Attitude'); legend('Roll','Pitch','Yaw');

    % Thrust
    subplot(2,3,5);
    plot(t, flight_data.thrust_cmd, 'k', 'LineWidth', 1.2);
    grid on; xlabel('Time [s]'); ylabel('Thrust [N]');
    title('Thrust Command');

    % Battery
    subplot(2,3,6);
    plot(t, flight_data.battery_soc * 100, 'm', 'LineWidth', 1.5);
    grid on; xlabel('Time [s]'); ylabel('SOC [%]');
    title('Battery'); ylim([0 105]);

end
