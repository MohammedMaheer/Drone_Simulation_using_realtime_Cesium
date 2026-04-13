function waypoints = mission_profiles(profile_name, params)
% MISSION_PROFILES  Pre-defined waypoint missions for testing.
%
%   waypoints = mission_profiles(profile_name, params)
%
%   Inputs:
%     profile_name - Mission name: 'hover', 'square', 'circle', 'figure8',
%                    'helix', 'landing', 'survey'
%     params       - struct with mission-specific overrides:
%                      .altitude   - Flight altitude [m] (default: 10)
%                      .size       - Pattern size [m] (default: 20)
%                      .num_points - Points for curved paths (default: 36)
%                      .yaw_mode   - 'fixed', 'heading', 'poi' (default: 'heading')
%
%   Outputs:
%     waypoints - [Nx4] matrix [x, y, z_ned, yaw]  (z_ned negative = up)

    if nargin < 2; params = struct(); end
    if ~isfield(params, 'altitude');   params.altitude   = 10;  end
    if ~isfield(params, 'size');       params.size       = 20;  end
    if ~isfield(params, 'num_points'); params.num_points = 36;  end
    if ~isfield(params, 'yaw_mode');   params.yaw_mode   = 'heading'; end

    alt = -params.altitude;  % NED convention
    sz  = params.size;

    switch lower(profile_name)
        case 'hover'
            % Simple takeoff and hover
            waypoints = [
                0,  0,  alt, 0;  % Hover at origin
            ];

        case 'square'
            % Fly a square pattern
            waypoints = [
                 sz/2,  sz/2, alt, 0;
                -sz/2,  sz/2, alt, pi/2;
                -sz/2, -sz/2, alt, pi;
                 sz/2, -sz/2, alt, -pi/2;
                 sz/2,  sz/2, alt, 0;       % Return to start
                 0,     0,    alt, 0;       % Return to origin
            ];

        case 'circle'
            N = params.num_points;
            theta = linspace(0, 2*pi, N+1)';
            theta = theta(1:end-1);
            R = sz / 2;
            x = R * cos(theta);
            y = R * sin(theta);
            z = alt * ones(N, 1);

            if strcmp(params.yaw_mode, 'heading')
                yaw = atan2(diff([y; y(1)]), diff([x; x(1)]));
            else
                yaw = zeros(N, 1);
            end
            waypoints = [x, y, z, yaw];

        case 'figure8'
            N = params.num_points;
            t = linspace(0, 2*pi, N)';
            R = sz / 2;
            x = R * sin(t);
            y = R * sin(2*t) / 2;
            z = alt * ones(N, 1);
            yaw = atan2(diff([y; y(1)]), diff([x; x(1)]));
            waypoints = [x, y, z, yaw];

        case 'helix'
            % Ascending helix
            N = params.num_points;
            turns = 3;
            t = linspace(0, turns * 2 * pi, N)';
            R = sz / 2;
            x = R * cos(t);
            y = R * sin(t);
            z = linspace(0, alt, N)';  % Climb from ground to altitude
            yaw = atan2(diff([y; y(1)]), diff([x; x(1)]));
            waypoints = [x, y, z, yaw];

        case 'landing'
            % Approach and land sequence
            waypoints = [
                0,   0,  alt,    0;       % Hover
                0,   0,  alt/2,  0;       % Descend halfway
                0,   0,  -2,     0;       % Low altitude
                0,   0,  0,      0;       % Touch down
            ];

        case 'survey'
            % Lawn-mower survey pattern
            num_legs = 5;
            leg_length = sz;
            leg_spacing = sz / num_legs;
            wps = [];
            for i = 0:num_legs-1
                y_pos = i * leg_spacing - sz/2;
                if mod(i, 2) == 0
                    wps = [wps; -leg_length/2, y_pos, alt, 0];
                    wps = [wps;  leg_length/2, y_pos, alt, 0];
                else
                    wps = [wps;  leg_length/2, y_pos, alt, pi];
                    wps = [wps; -leg_length/2, y_pos, alt, pi];
                end
            end
            waypoints = wps;

        otherwise
            error('Unknown mission profile: %s', profile_name);
    end

    % Add takeoff as first waypoint if starting from ground
    if waypoints(1,3) < -1
        takeoff_wp = [0, 0, waypoints(1,3), 0];
        waypoints = [takeoff_wp; waypoints];
    end

end
