function [thrust_cmd, alt_error] = altitude_controller(z_current, vz_current, z_desired, dp, cp)
% ALTITUDE_CONTROLLER  PID altitude hold controller.
%
%   [thrust_cmd, alt_error] = altitude_controller(z_current, vz_current, z_desired, dp, cp)
%
%   Inputs:
%     z_current  - Current NED z position [m] (negative = up)
%     vz_current - Current NED z velocity [m/s] (negative = climbing)
%     z_desired  - Desired NED z position [m]
%     dp         - drone_params struct
%     cp         - controller_params struct
%
%   Outputs:
%     thrust_cmd - Total thrust command [N]
%     alt_error  - Altitude error [m]

    persistent int_error prev_error
    if isempty(int_error)
        int_error  = 0;
        prev_error = 0;
    end

    %% Error (NED: negative z = up, so error sign is correct)
    alt_error = z_desired - z_current;

    %% PID gains
    Kp = cp.alt.Kp;
    Ki = cp.alt.Ki;
    Kd = cp.alt.Kd;

    dt = 1 / cp.position_rate;

    %% PID computation
    P = Kp * alt_error;

    int_error = int_error + alt_error * dt;
    int_error = max(-cp.integrator_max, min(cp.integrator_max, int_error));
    I = Ki * int_error;

    D = Kd * (alt_error - prev_error) / dt;
    prev_error = alt_error;

    %% Thrust command
    % Feedforward: hover thrust compensates gravity
    thrust_ff = dp.mass * dp.g;

    % PID correction (negative because NED z is down)
    thrust_cmd = thrust_ff - (P + I + D);

    % Saturate
    thrust_cmd = max(cp.alt.min_thrust, min(cp.alt.max_thrust, thrust_cmd));

end
