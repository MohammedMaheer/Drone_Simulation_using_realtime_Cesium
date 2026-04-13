classdef telemetry_logger < handle
% TELEMETRY_LOGGER  Records simulation data for post-flight analysis.
%
%   logger = telemetry_logger(max_samples)
%   logger = telemetry_logger(max_samples, num_motors)
%   logger.log(t, state, ctrl_data, sensor_data, wp_status)
%   data = logger.get_data()
%   logger.save('filename.mat')

    properties
        time
        position        % [Nx3]
        velocity        % [Nx3]
        euler           % [Nx3]  roll, pitch, yaw
        omega           % [Nx3]  body angular rates
        motor_speeds    % [NxM]  M = num_motors
        thrust_cmd      % [Nx1]
        moment_cmds     % [Nx3]
        pos_error       % [Nx3]
        att_error       % [Nx3]
        desired_euler   % [Nx3]
        gps_pos         % [Nx3]
        baro_alt        % [Nx1]
        est_pos         % [Nx3]  estimated position
        battery_soc     % [Nx1]  state of charge
        wp_index        % [Nx1]  current waypoint
        wp_distance     % [Nx1]  distance to waypoint
        wind            % [Nx3]
        idx             % Current write index
        max_samples
        num_motors      % Number of motors
    end

    methods
        function obj = telemetry_logger(max_samples, num_motors)
            if nargin < 1; max_samples = 100000; end
            if nargin < 2; num_motors = 4; end
            obj.max_samples = max_samples;
            obj.num_motors  = num_motors;
            obj.idx = 0;

            % Pre-allocate arrays
            obj.time         = zeros(max_samples, 1);
            obj.position     = zeros(max_samples, 3);
            obj.velocity     = zeros(max_samples, 3);
            obj.euler        = zeros(max_samples, 3);
            obj.omega        = zeros(max_samples, 3);
            obj.motor_speeds = zeros(max_samples, num_motors);
            obj.thrust_cmd   = zeros(max_samples, 1);
            obj.moment_cmds  = zeros(max_samples, 3);
            obj.pos_error    = zeros(max_samples, 3);
            obj.att_error    = zeros(max_samples, 3);
            obj.desired_euler = zeros(max_samples, 3);
            obj.gps_pos      = zeros(max_samples, 3);
            obj.baro_alt     = zeros(max_samples, 1);
            obj.est_pos      = zeros(max_samples, 3);
            obj.battery_soc  = zeros(max_samples, 1);
            obj.wp_index     = zeros(max_samples, 1);
            obj.wp_distance  = zeros(max_samples, 1);
            obj.wind         = zeros(max_samples, 3);
        end

        function log(obj, t, state, motor_spd, ctrl_data, sensor_data, est_state, batt_soc, wp_status, wind_vec)
        % LOG  Record one time step of telemetry data.
            obj.idx = obj.idx + 1;
            i = obj.idx;

            if i > obj.max_samples
                warning('Telemetry buffer full, stopping logging.');
                return;
            end

            obj.time(i)           = t;
            obj.position(i,:)     = state(1:3)';
            obj.velocity(i,:)     = state(4:6)';
            obj.euler(i,:)        = state(7:9)';
            obj.omega(i,:)        = state(10:12)';
            obj.motor_speeds(i,:) = motor_spd';

            if ~isempty(ctrl_data)
                obj.thrust_cmd(i)      = ctrl_data.thrust_cmd;
                obj.moment_cmds(i,:)   = ctrl_data.moment_cmds';
                obj.pos_error(i,:)     = ctrl_data.pos_error';
                obj.att_error(i,:)     = ctrl_data.att_error';
                obj.desired_euler(i,:) = ctrl_data.desired_euler';
            end

            if ~isempty(sensor_data) && isfield(sensor_data, 'gps_pos')
                obj.gps_pos(i,:) = sensor_data.gps_pos';
            end
            if ~isempty(sensor_data) && isfield(sensor_data, 'baro_alt')
                obj.baro_alt(i) = sensor_data.baro_alt;
            end

            if ~isempty(est_state)
                obj.est_pos(i,:) = est_state(1:3)';
            end

            obj.battery_soc(i) = batt_soc;

            if ~isempty(wp_status)
                obj.wp_index(i)    = wp_status.current_idx;
                obj.wp_distance(i) = wp_status.distance;
            end

            if ~isempty(wind_vec)
                obj.wind(i,:) = wind_vec';
            end
        end

        function data = get_data(obj)
        % GET_DATA  Return trimmed telemetry as a struct.
            n = obj.idx;
            data.time         = obj.time(1:n);
            data.position     = obj.position(1:n,:);
            data.velocity     = obj.velocity(1:n,:);
            data.euler        = obj.euler(1:n,:);
            data.omega        = obj.omega(1:n,:);
            data.motor_speeds = obj.motor_speeds(1:n,:);
            data.thrust_cmd   = obj.thrust_cmd(1:n);
            data.moment_cmds  = obj.moment_cmds(1:n,:);
            data.pos_error    = obj.pos_error(1:n,:);
            data.att_error    = obj.att_error(1:n,:);
            data.desired_euler = obj.desired_euler(1:n,:);
            data.gps_pos      = obj.gps_pos(1:n,:);
            data.baro_alt     = obj.baro_alt(1:n);
            data.est_pos      = obj.est_pos(1:n,:);
            data.battery_soc  = obj.battery_soc(1:n);
            data.wp_index     = obj.wp_index(1:n);
            data.wp_distance  = obj.wp_distance(1:n);
            data.wind         = obj.wind(1:n,:);
        end

        function save_log(obj, filename)
        % SAVE_LOG  Save telemetry to a .mat file.
            if nargin < 2; filename = 'flight_log.mat'; end
            flight_data = obj.get_data();  %#ok<NASGU>
            save(filename, 'flight_data');
            fprintf('Flight log saved to: %s (%d samples)\n', filename, obj.idx);
        end
    end
end
