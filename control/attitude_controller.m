function [desired_rates, att_error] = attitude_controller(euler_current, euler_desired, cp)
% ATTITUDE_CONTROLLER  PID attitude controller for roll, pitch, and yaw.
%
%   [desired_rates, att_error] = attitude_controller(euler_current, euler_desired, cp)
%
%   Inputs:
%     euler_current  - [3x1] current Euler angles [phi; theta; psi] [rad]
%     euler_desired  - [3x1] desired Euler angles [phi; theta; psi] [rad]
%     cp             - controller_params struct
%
%   Outputs:
%     desired_rates - [3x1] desired body angular rates [p; q; r] [rad/s]
%     att_error     - [3x1] attitude error [rad]

    persistent int_error prev_measurement
    if isempty(int_error)
        int_error        = [0; 0; 0];
        prev_measurement = [0; 0; 0];
    end

    %% Compute error
    att_error = euler_desired - euler_current;

    % Wrap yaw error to [-pi, pi]
    att_error(3) = atan2(sin(att_error(3)), cos(att_error(3)));

    %% PID gains
    Kp = [cp.roll.Kp;  cp.pitch.Kp;  cp.yaw.Kp];
    Ki = [cp.roll.Ki;  cp.pitch.Ki;  cp.yaw.Ki];
    Kd = [cp.roll.Kd;  cp.pitch.Kd;  cp.yaw.Kd];

    %% PID computation
    dt = 1 / cp.attitude_rate;

    % Proportional
    P = Kp .* att_error;

    % Integral with anti-windup
    int_error = int_error + att_error * dt;
    int_error = max(-cp.integrator_max, min(cp.integrator_max, int_error));
    I = Ki .* int_error;

    % Derivative-on-measurement (avoids derivative kick on setpoint changes)
    % D = -Kd * d(measurement)/dt  instead of  D = Kd * d(error)/dt
    meas_diff = euler_current - prev_measurement;
    % Wrap yaw measurement difference to [-pi, pi]
    meas_diff(3) = atan2(sin(meas_diff(3)), cos(meas_diff(3)));
    D = -Kd .* meas_diff / dt;
    prev_measurement = euler_current;

    %% Output with rate limiting
    desired_rates = P + I + D;
    desired_rates(1) = max(-cp.max_roll_rate,  min(cp.max_roll_rate,  desired_rates(1)));
    desired_rates(2) = max(-cp.max_pitch_rate, min(cp.max_pitch_rate, desired_rates(2)));
    desired_rates(3) = max(-cp.max_yaw_rate,   min(cp.max_yaw_rate,   desired_rates(3)));

end
