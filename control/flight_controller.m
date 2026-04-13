function [thrust_cmd, moment_cmds, ctrl_data] = flight_controller(state, target, dp, cp)
% FLIGHT_CONTROLLER  Unified cascaded flight controller.
%
%   [thrust_cmd, moment_cmds, ctrl_data] = flight_controller(state, target, dp, cp)
%
%   Implements the full cascaded control loop:
%     Position Controller → Attitude Controller → Rate Controller → Mixing
%
%   Inputs:
%     state  - [12x1] state vector [x y z vx vy vz phi theta psi p q r]'
%     target - struct with fields:
%                .position  - [3x1] desired position NED [m]
%                .yaw       - desired yaw angle [rad]
%     dp     - drone_params struct
%     cp     - controller_params struct
%
%   Outputs:
%     thrust_cmd  - Total thrust command [N]
%     moment_cmds - [3x1] body moment commands [tau_x; tau_y; tau_z] [N·m]
%     ctrl_data   - struct with intermediate controller data for telemetry

    %% Unpack state
    pos   = state(1:3);
    vel   = state(4:6);
    euler = state(7:9);
    omega = state(10:12);

    %% 1. Position Controller (outer loop)
    [desired_euler, pos_error] = position_controller(...
        pos, vel, target.position, target.yaw, cp);

    %% 2. Altitude Controller
    [thrust_cmd, alt_error] = altitude_controller(...
        pos(3), vel(3), target.position(3), dp, cp);

    %% 3. Attitude Controller (middle loop)
    [desired_rates, att_error] = attitude_controller(...
        euler, desired_euler, cp);

    %% 4. Rate Controller (inner loop)
    moment_cmds = rate_controller(omega, desired_rates, cp);

    %% Pack telemetry data
    ctrl_data.pos_error      = pos_error;
    ctrl_data.alt_error      = alt_error;
    ctrl_data.att_error      = att_error;
    ctrl_data.desired_euler  = desired_euler;
    ctrl_data.desired_rates  = desired_rates;
    ctrl_data.thrust_cmd     = thrust_cmd;
    ctrl_data.moment_cmds    = moment_cmds;

end


function moment_cmds = rate_controller(omega_current, omega_desired, cp)
% Rate controller (innermost loop) — full PID on angular rates

    persistent prev_error int_error
    if isempty(prev_error)
        prev_error = [0; 0; 0];
        int_error  = [0; 0; 0];
    end

    rate_error = omega_desired - omega_current;

    Kp = [cp.roll_rate.Kp;  cp.pitch_rate.Kp;  cp.yaw_rate.Kp];
    Ki = [cp.roll_rate.Ki;  cp.pitch_rate.Ki;  cp.yaw_rate.Ki];
    Kd = [cp.roll_rate.Kd;  cp.pitch_rate.Kd;  cp.yaw_rate.Kd];

    dt = 1 / cp.rate_rate;

    P = Kp .* rate_error;

    % Integral with anti-windup
    int_error = int_error + rate_error * dt;
    int_max = 0.5;  % Tight limit for rate loop integrator
    int_error = max(-int_max, min(int_max, int_error));
    I = Ki .* int_error;

    D = Kd .* (rate_error - prev_error) / dt;
    prev_error = rate_error;

    moment_cmds = P + I + D;
end
