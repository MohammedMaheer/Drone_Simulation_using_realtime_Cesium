function [current_target, wp_status] = waypoint_manager(position, waypoints, wp_params, dt)
% WAYPOINT_MANAGER  Manages waypoint sequencing and acceptance logic.
%
%   [current_target, wp_status] = waypoint_manager(position, waypoints, wp_params, dt)
%
%   Inputs:
%     position  - [3x1] current position NED [m]
%     waypoints - [Nx4] matrix, each row: [x, y, z_ned, yaw] 
%     wp_params - struct with fields:
%                   .accept_radius  - Distance to accept waypoint [m]
%                   .accept_alt     - Altitude tolerance [m]
%                   .loiter_time    - Time to hold at waypoint [s]
%     dt        - simulation timestep [s] (default: 0.001)
%
%   Outputs:
%     current_target - struct with .position [3x1] and .yaw
%     wp_status      - struct with .current_idx, .completed, .distance, .total

    persistent wp_idx arrived_time
    if isempty(wp_idx)
        wp_idx = 1;
        arrived_time = -1;
    end

    if nargin < 4 || isempty(dt); dt = 0.001; end

    num_wp = size(waypoints, 1);

    %% Default params
    if nargin < 3 || isempty(wp_params)
        wp_params.accept_radius = 1.5;  % [m]
        wp_params.accept_alt    = 0.8;  % [m]
        wp_params.loiter_time   = 2.0;  % [s]
    end

    %% Check if mission complete
    if wp_idx > num_wp
        % Hold last waypoint
        wp = waypoints(num_wp, :);
        current_target.position = wp(1:3)';
        current_target.yaw      = wp(4);
        wp_status.current_idx   = num_wp;
        wp_status.completed     = true;
        wp_status.distance      = 0;
        wp_status.total         = num_wp;
        return;
    end

    %% Current target waypoint
    wp = waypoints(wp_idx, :);
    target_pos = wp(1:3)';
    target_yaw = wp(4);

    %% Distance to current waypoint
    dist_horiz = norm(position(1:2) - target_pos(1:2));
    dist_vert  = abs(position(3) - target_pos(3));

    %% Waypoint acceptance logic
    if dist_horiz < wp_params.accept_radius && dist_vert < wp_params.accept_alt
        if arrived_time < 0
            arrived_time = 0;  % Mark arrival (will be incremented externally)
        end
        arrived_time = arrived_time + dt;  % Accumulate elapsed time at waypoint

        if arrived_time >= wp_params.loiter_time
            % Advance to next waypoint
            wp_idx = wp_idx + 1;
            arrived_time = -1;

            if wp_idx <= num_wp
                wp = waypoints(wp_idx, :);
                target_pos = wp(1:3)';
                target_yaw = wp(4);
            end
        end
    else
        arrived_time = -1;
    end

    %% Build output
    current_target.position = target_pos;
    current_target.yaw      = target_yaw;

    wp_status.current_idx = min(wp_idx, num_wp);
    wp_status.completed   = wp_idx > num_wp;
    wp_status.distance    = norm(position - target_pos);
    wp_status.total       = num_wp;

end
