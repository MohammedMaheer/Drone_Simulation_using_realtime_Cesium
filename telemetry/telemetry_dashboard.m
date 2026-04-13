function telemetry_dashboard(state, ctrl_data, batt_soc, wp_status, t)
% TELEMETRY_DASHBOARD  Real-time telemetry display during simulation.
%
%   telemetry_dashboard(state, ctrl_data, batt_soc, wp_status, t)
%
%   Creates/updates a live dashboard figure showing attitude, position,
%   altitude, motor status, and navigation info.

    persistent fig_handle h_text
    persistent h_roll h_pitch h_yaw h_pos h_pos_dot h_alt
    persistent t_buf roll_buf pitch_buf yaw_buf x_buf y_buf alt_t_buf alt_buf
    persistent buf_idx max_buf

    if isempty(fig_handle) || ~isvalid(fig_handle)
        max_buf = 3000;  % Pre-allocated buffer size
        buf_idx = 0;
        t_buf = NaN(max_buf,1); roll_buf = NaN(max_buf,1);
        pitch_buf = NaN(max_buf,1); yaw_buf = NaN(max_buf,1);
        x_buf = NaN(max_buf,1); y_buf = NaN(max_buf,1);
        alt_t_buf = NaN(max_buf,1); alt_buf = NaN(max_buf,1);

        fig_handle = figure('Name', 'Drone Telemetry Dashboard', ...
            'NumberTitle', 'off', 'Position', [50 50 1400 750]);

        % Attitude subplot
        ax_att = subplot(2,3,1); hold on; grid on;
        title('Attitude [deg]'); xlabel('Time [s]'); ylabel('Angle [deg]');
        h_roll  = animatedline(ax_att, 'Color','r', 'MaximumNumPoints',max_buf);
        h_pitch = animatedline(ax_att, 'Color','g', 'MaximumNumPoints',max_buf);
        h_yaw   = animatedline(ax_att, 'Color','b', 'MaximumNumPoints',max_buf);
        legend('Roll','Pitch','Yaw', 'Location', 'northwest');

        % Position XY subplot
        ax_pos = subplot(2,3,2); hold on; grid on; axis equal;
        title('Position XY [m]'); xlabel('X North [m]'); ylabel('Y East [m]');
        h_pos     = animatedline(ax_pos, 'Color','b', 'MaximumNumPoints',max_buf);
        h_pos_dot = line(ax_pos, NaN, NaN, 'Marker','o', 'Color','r', ...
            'MarkerSize',8, 'MarkerFaceColor','r');

        % Altitude subplot
        ax_alt = subplot(2,3,3); hold on; grid on;
        title('Altitude [m]'); xlabel('Time [s]'); ylabel('Alt [m]');
        h_alt = animatedline(ax_alt, 'Color','b', 'MaximumNumPoints',max_buf);

        % Text info panel
        subplot(2,3,4:6); axis off;
        h_text = text(0.05, 0.5, '', 'FontName', 'Courier', 'FontSize', 10, ...
            'VerticalAlignment', 'middle', 'Units', 'normalized');
    end

    %% Append data using animatedline (fast, no cla)
    euler_deg = rad2deg(state(7:9));
    addpoints(h_roll,  t, euler_deg(1));
    addpoints(h_pitch, t, euler_deg(2));
    addpoints(h_yaw,   t, euler_deg(3));

    addpoints(h_pos, state(1), state(2));
    set(h_pos_dot, 'XData', state(1), 'YData', state(2));

    altitude = -state(3);
    addpoints(h_alt, t, altitude);

    %% Update text info
    speed = norm(state(4:6));
    info_str = sprintf([...
        'TIME: %6.1f s  |  BATTERY: %5.1f%%\n', ...
        '─────────────────────────────────────\n', ...
        'POSITION:  X=%7.2f  Y=%7.2f  Z=%7.2f m\n', ...
        'VELOCITY:  Vx=%6.2f  Vy=%6.2f  Vz=%6.2f m/s  |  Speed=%5.2f m/s\n', ...
        'ATTITUDE:  Roll=%6.1f°  Pitch=%6.1f°  Yaw=%6.1f°\n', ...
        'ALTITUDE:  %6.2f m\n'], ...
        t, batt_soc*100, ...
        state(1), state(2), state(3), ...
        state(4), state(5), state(6), speed, ...
        euler_deg(1), euler_deg(2), euler_deg(3), ...
        altitude);

    if ~isempty(wp_status)
        info_str = [info_str, sprintf(...
            'WAYPOINT:  %d/%d  |  Distance: %5.1f m  |  %s\n', ...
            wp_status.current_idx, wp_status.total, wp_status.distance, ...
            ternary(wp_status.completed, 'MISSION COMPLETE', 'IN PROGRESS'))];
    end

    if ~isempty(ctrl_data)
        info_str = [info_str, sprintf(...
            'THRUST:    %6.2f N  |  Moments: [%6.4f, %6.4f, %6.4f] N·m\n', ...
            ctrl_data.thrust_cmd, ctrl_data.moment_cmds(1), ...
            ctrl_data.moment_cmds(2), ctrl_data.moment_cmds(3))];
    end

    set(h_text, 'String', info_str);
    drawnow limitrate;
end


function result = ternary(condition, true_val, false_val)
    if condition
        result = true_val;
    else
        result = false_val;
    end
end
