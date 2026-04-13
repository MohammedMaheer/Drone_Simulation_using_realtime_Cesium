function [pos_gps, vel_gps, fix_valid] = gps_model(pos_true, vel_true, t, sp)
% GPS_MODEL  Simulates GPS position and velocity measurements.
%
%   [pos_gps, vel_gps, fix_valid] = gps_model(pos_true, vel_true, t, sp)
%
%   Inputs:
%     pos_true - [3x1] true position NED [m]
%     vel_true - [3x1] true velocity NED [m/s]
%     t        - current sim time [s]
%     sp       - sensor_params struct
%
%   Outputs:
%     pos_gps   - [3x1] GPS position measurement NED [m]
%     vel_gps   - [3x1] GPS velocity measurement NED [m/s]
%     fix_valid - boolean, true if GPS fix is valid

    persistent last_update_time last_pos last_vel
    if isempty(last_update_time)
        last_update_time = -1;
        last_pos = pos_true;
        last_vel = vel_true;
    end

    %% Check update rate
    update_period = 1 / sp.gps.update_rate;
    if (t - last_update_time) >= update_period
        last_update_time = t;

        % Position noise (scaled by HDOP for horizontal)
        pos_noise = sp.gps.pos_noise_std * sp.gps.hdop * randn(3,1);
        pos_noise(3) = pos_noise(3) * 1.5;  % Vertical accuracy worse

        % Velocity noise
        vel_noise = sp.gps.vel_noise_std * randn(3,1);

        last_pos = pos_true + pos_noise;
        last_vel = vel_true + vel_noise;
    end

    pos_gps = last_pos;
    vel_gps = last_vel;

    % Simple fix validity (always valid in sim, but can be toggled)
    fix_valid = true;

end
