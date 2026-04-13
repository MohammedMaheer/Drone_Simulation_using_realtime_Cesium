function [desired_euler, pos_error] = position_controller(pos_current, vel_current, pos_desired, yaw_desired, cp)
% POSITION_CONTROLLER  PID position controller producing desired roll/pitch.
%
%   [desired_euler, pos_error] = position_controller(pos_current, vel_current, pos_desired, yaw_desired, cp)
%
%   Inputs:
%     pos_current  - [3x1] current position [x; y; z] NED [m]
%     vel_current  - [3x1] current velocity [vx; vy; vz] NED [m/s]
%     pos_desired  - [3x1] desired position [x; y; z] NED [m]
%     yaw_desired  - desired yaw angle [rad]
%     cp           - controller_params struct
%
%   Outputs:
%     desired_euler - [3x1] desired [roll; pitch; yaw] [rad]
%     pos_error     - [3x1] position error [m]

    persistent int_error_xy prev_error_xy
    if isempty(int_error_xy)
        int_error_xy  = [0; 0];
        prev_error_xy = [0; 0];
    end

    %% Position error (only X-Y, altitude is separate)
    pos_error = pos_desired - pos_current;
    error_xy = pos_error(1:2);

    %% PID gains
    Kp = [cp.pos_x.Kp; cp.pos_y.Kp];
    Ki = [cp.pos_x.Ki; cp.pos_y.Ki];
    Kd = [cp.pos_x.Kd; cp.pos_y.Kd];

    dt = 1 / cp.position_rate;

    %% PID computation (NED frame)
    P = Kp .* error_xy;

    int_error_xy = int_error_xy + error_xy * dt;
    int_error_xy = max(-cp.integrator_max, min(cp.integrator_max, int_error_xy));
    I = Ki .* int_error_xy;

    D = Kd .* (error_xy - prev_error_xy) / dt;
    prev_error_xy = error_xy;

    % Desired acceleration in NED (X-Y)
    accel_desired = P + I + D;

    %% Convert NED acceleration to desired roll & pitch
    % From dynamics: F_x = -T*sin(theta) → a_x = -g*theta (small angles)
    %                F_y =  T*sin(phi)   → a_y =  g*phi
    % So: theta = -a_x/g, phi = a_y/g
    % With yaw rotation to body frame:
    cpsi = cos(yaw_desired);
    spsi = sin(yaw_desired);

    % Rotate to body frame
    ax_body =  cpsi * accel_desired(1) + spsi * accel_desired(2);
    ay_body = -spsi * accel_desired(1) + cpsi * accel_desired(2);

    g = 9.81;
    desired_pitch = atan2(-ax_body, g);
    desired_roll  = atan2(ay_body, g);

    % Saturate angles
    max_angle = cp.pos_x.max_angle;
    desired_roll  = max(-max_angle, min(max_angle, desired_roll));
    desired_pitch = max(-max_angle, min(max_angle, desired_pitch));

    desired_euler = [desired_roll; desired_pitch; yaw_desired];

end
