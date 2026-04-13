function [omega_out, thrust, torque, current] = motor_model(omega_cmd, omega_current, dt, dp)
% MOTOR_MODEL  First-order motor dynamics with saturation and electrical model.
%
%   [omega_out, thrust, torque, current] = motor_model(omega_cmd, omega_current, dt, dp)
%
%   Inputs:
%     omega_cmd     - [4x1] commanded motor speeds [rad/s]
%     omega_current - [4x1] current motor speeds [rad/s]
%     dt            - time step [s]
%     dp            - drone_params struct
%
%   Outputs:
%     omega_out - [4x1] updated motor speeds [rad/s]
%     thrust    - [4x1] thrust per motor [N]
%     torque    - [4x1] reaction torque per motor [N·m]
%     current   - [4x1] motor current draw [A]

    %% Saturate commanded speed
    omega_cmd = max(rpm2rads(dp.omega_min), min(rpm2rads(dp.omega_max), omega_cmd));

    %% First-order motor dynamics
    % omega_dot = (omega_cmd - omega_current) / tau
    alpha = dt / (dp.tau_motor + dt);  % Discrete-time filter coefficient
    omega_out = omega_current + alpha * (omega_cmd - omega_current);

    %% Compute thrust and torque
    thrust = dp.kT * omega_out.^2;
    torque = dp.kQ * omega_out.^2;

    %% Electrical model (simplified)
    % P_mech = torque * omega, P_elec = V * I
    P_mech  = torque .* abs(omega_out);
    eta     = 0.85;  % Motor efficiency
    current = P_mech / (eta * dp.V_battery);

end


function rads = rpm2rads(rpm)
    rads = rpm * 2 * pi / 60;
end
