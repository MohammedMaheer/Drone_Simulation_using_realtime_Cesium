function motor_cmds = mixing_matrix_n(thrust_cmd, moment_cmds, dp)
% MIXING_MATRIX_N  General N-motor mixing for any multirotor configuration.
%
%   motor_cmds = mixing_matrix_n(thrust_cmd, moment_cmds, dp)
%
%   Inputs:
%     thrust_cmd  - Desired total thrust [N]
%     moment_cmds - [3x1] desired body moments [tau_x; tau_y; tau_z] [N*m]
%     dp          - drone config struct (must have dp.mix_matrix, dp.num_motors,
%                   dp.kT, dp.omega_min, dp.omega_max)
%
%   Outputs:
%     motor_cmds  - [Nx1] motor speed commands [rad/s]
%
%   Uses the precomputed mix_matrix from drone_config or drone_params, and
%   solves via pseudoinverse for N>4 (overdetermined) or direct inverse for N=4.

    n = dp.num_motors;
    A = dp.mix_matrix;   % [4 x N] allocation matrix

    % Desired wrench: [Total Thrust; Roll Torque; Pitch Torque; Yaw Torque]
    wrench = [thrust_cmd; moment_cmds];

    % Solve for individual motor thrusts
    if n == 4
        % Square system: direct solve
        T_motors = A \ wrench;
    else
        % Over/under-determined: minimum-norm pseudoinverse
        % For N>4, this distributes load evenly across motors
        % For N<4 (tricopter), uses least-squares fit
        T_motors = pinv(A) * wrench;
    end

    % Clamp negative thrusts to zero
    T_motors = max(T_motors, 0);

    % Convert thrust to motor speed: T = kT * omega^2  =>  omega = sqrt(T/kT)
    motor_cmds = sqrt(T_motors / dp.kT);

    % Saturate motor speeds
    omega_min = dp.omega_min * 2 * pi / 60;
    omega_max = dp.omega_max * 2 * pi / 60;
    motor_cmds = max(omega_min, min(omega_max, motor_cmds));
end
