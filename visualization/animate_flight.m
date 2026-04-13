function animate_flight(flight_data, speed_factor, save_video, drone_cfg)
% ANIMATE_FLIGHT  Animated 3D replay of a recorded flight.
%
%   animate_flight(flight_data)
%   animate_flight(flight_data, 2.0)          % 2x speed
%   animate_flight(flight_data, 1.0, true)    % Save as video
%   animate_flight(flight_data, 1.0, false, cfg)  % Use drone_config
%
%   Inputs:
%     flight_data  - Struct from telemetry_logger.get_data()
%     speed_factor - Playback speed multiplier (default: 1.0)
%     save_video   - Save animation as .avi video (default: false)
%     drone_cfg    - (optional) drone config struct from drone_config()
%                    Used for arm_length and motor_layout

    if nargin < 2; speed_factor = 1.0; end
    if nargin < 3; save_video = false; end
    if nargin < 4; drone_cfg = []; end

    % Extract arm_length and motor_layout from config if provided
    if ~isempty(drone_cfg) && isfield(drone_cfg, 'drone')
        arm_len = drone_cfg.drone.arm_length;
        motor_layout = drone_cfg.motor_layout;
    elseif ~isempty(drone_cfg) && isfield(drone_cfg, 'arm_length')
        arm_len = drone_cfg.arm_length;
        motor_layout = [];
    else
        arm_len = 0.23;
        motor_layout = [];
    end

    t   = flight_data.time;
    pos = flight_data.position;
    eul = flight_data.euler;
    alt = -pos(:,3);

    %% Setup figure
    fig = figure('Name', 'Flight Animation', 'Position', [100 100 1200 800]);

    % 3D view
    ax3d = subplot(1,2,1);
    hold on; grid on; axis equal;
    xlabel('X North [m]'); ylabel('Y East [m]'); zlabel('Altitude [m]');
    title('Flight Animation');
    view(30, 25);

    % Set axis limits with padding
    pad = 5;
    xlim([min(pos(:,1))-pad, max(pos(:,1))+pad]);
    ylim([min(pos(:,2))-pad, max(pos(:,2))+pad]);
    zlim([0, max(alt)+pad]);

    % Lighting for 3D geometry
    set(ax3d, 'AmbientLightColor', [0.35 0.35 0.4]);
    light(ax3d, 'Position', [1 0.5 -1], 'Style', 'infinite', 'Color', [1.0 0.95 0.85]);
    light(ax3d, 'Position', [-0.5 -1 -0.5], 'Style', 'infinite', 'Color', [0.25 0.3 0.45]);
    lighting(ax3d, 'gouraud');

    % Draw ground plane
    ground_x = xlim; ground_y = ylim;
    patch([ground_x(1) ground_x(2) ground_x(2) ground_x(1)], ...
          [ground_y(1) ground_y(1) ground_y(2) ground_y(2)], ...
          [0 0 0 0], [0.4 0.6 0.3], 'FaceAlpha', 0.5, 'EdgeColor', 'none', ...
          'FaceLighting', 'gouraud');

    % Check if motor_speeds data is available
    has_motor_data = isfield(flight_data, 'motor_speeds') && ...
                     ~isempty(flight_data.motor_speeds);
    anim_prop_angle = 0;

    % Trail line
    trail = plot3(NaN, NaN, NaN, 'b-', 'LineWidth', 1.5);

    % Info panel
    ax_info = subplot(1,2,2);
    axis off;
    info_text = text(0.05, 0.8, '', 'FontName', 'Courier', 'FontSize', 11, ...
        'VerticalAlignment', 'top', 'Units', 'normalized');

    %% Video writer
    if save_video
        vid = VideoWriter('flight_animation.avi');
        vid.FrameRate = 30;
        open(vid);
    end

    %% Animation loop
    dt_anim = 1/30;  % 30 FPS
    dt_sim  = t(2) - t(1);
    step = max(1, round(dt_anim / dt_sim * speed_factor));

    trail_x = []; trail_y = []; trail_z = [];

    for i = 1:step:length(t)
        if ~isvalid(fig); break; end

        % Clear and redraw drone
        axes(ax3d);
        delete(findobj(ax3d, 'Tag', 'drone_part'));

        % Draw drone with realistic 3D geometry
        state_pos = pos(i,:)';
        state_eul = eul(i,:)';
        if has_motor_data
            m_spd = flight_data.motor_speeds(i,:)';
            anim_prop_angle = anim_prop_angle + mean(abs(m_spd)) * dt_anim * 2;
        else
            m_spd = [];
        end
        h = drone_3d_plot(state_pos, state_eul, arm_len, ax3d, ...
            motor_layout, m_spd, anim_prop_angle);

        % Tag drone parts for cleanup
        fields = fieldnames(h);
        for f = 1:length(fields)
            objs = h.(fields{f});
            for o = 1:length(objs)
                set(objs(o), 'Tag', 'drone_part');
            end
        end

        % Update trail
        trail_x = [trail_x; pos(i,1)];
        trail_y = [trail_y; pos(i,2)];
        trail_z = [trail_z; alt(i)];
        set(trail, 'XData', trail_x, 'YData', trail_y, 'ZData', trail_z);

        % Shadow on ground
        delete(findobj(ax3d, 'Tag', 'shadow'));
        plot3(pos(i,1), pos(i,2), 0, 'k.', 'MarkerSize', 15, 'Tag', 'shadow');

        % Update info
        speed = norm(flight_data.velocity(i,:));
        info_str = sprintf([...
            'TIME:     %6.1f / %.1f s\n\n', ...
            'POSITION:\n', ...
            '  X: %7.2f m\n', ...
            '  Y: %7.2f m\n', ...
            '  Alt: %5.2f m\n\n', ...
            'SPEED:    %5.2f m/s\n\n', ...
            'ATTITUDE:\n', ...
            '  Roll:  %6.1f°\n', ...
            '  Pitch: %6.1f°\n', ...
            '  Yaw:   %6.1f°\n\n', ...
            'BATTERY:  %.0f%%\n\n', ...
            'WAYPOINT: %d'], ...
            t(i), t(end), ...
            pos(i,1), pos(i,2), alt(i), speed, ...
            rad2deg(eul(i,1)), rad2deg(eul(i,2)), rad2deg(eul(i,3)), ...
            flight_data.battery_soc(i)*100, ...
            flight_data.wp_index(i));
        set(info_text, 'String', info_str);

        drawnow limitrate;

        if save_video
            frame = getframe(fig);
            writeVideo(vid, frame);
        end
    end

    if save_video
        close(vid);
        fprintf('Video saved: flight_animation.avi\n');
    end

    fprintf('Animation complete.\n');
end
