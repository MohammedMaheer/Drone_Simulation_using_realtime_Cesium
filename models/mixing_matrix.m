function motor_cmds = mixing_matrix(thrust_cmd, moment_cmds, dp)
% MIXING_MATRIX  Convert desired thrust & moments to individual motor speed commands.
%
%   motor_cmds = mixing_matrix(thrust_cmd, moment_cmds, dp)
%
%   Inputs:
%     thrust_cmd  - Desired total thrust [N]
%     moment_cmds - [3x1] desired body moments [tau_x; tau_y; tau_z] [N·m]
%     dp          - drone_params struct
%
%   Outputs:
%     motor_cmds - [4x1] motor speed commands [rad/s]
%
%   Motor Configuration (X-quad, viewed from above):
%       3 (CCW)   1 (CW)       Front
%           \     /
%            \   /
%             [X]
%            /   \
%           /     \
%       2 (CW)   4 (CCW)       Rear
%
%   The mixing matrix M maps motor thrust squares to [T; tau_x; tau_y; tau_z]:
%     [T    ]     [  1        1        1        1    ] [T1]
%     [tau_x]  =  [ -Ls      +Ls      +Ls      -Ls   ] [T2]
%     [tau_y]     [ +Ls      -Ls      +Ls      -Ls   ] [T3]
%     [tau_z]     [ -kQ/kT   -kQ/kT   +kQ/kT   +kQ/kT] [T4]

    L  = dp.arm_length;
    kT = dp.kT;
    kQ = dp.kQ;
    s45 = sqrt(2) / 2;
    Ls = L * s45;

    % Allocation matrix: [T; tau_x; tau_y; tau_z] = A * [T1; T2; T3; T4]
    %   Roll  depends on motor y-position (opposite sides → opposite sign)
    %   Pitch depends on motor x-position
    %   Yaw   depends on spin direction (CW=-c, CCW=+c)
    c = kQ / kT;  % Torque-to-thrust ratio
    A = [ 1,     1,     1,     1;
         -Ls,    Ls,    Ls,   -Ls;
          Ls,   -Ls,    Ls,   -Ls;
         -c,    -c,     c,     c];

    % Desired wrench
    wrench = [thrust_cmd; moment_cmds];

    % Solve for individual motor thrusts
    T_motors = A \ wrench;  % [4x1] motor thrusts

    % Clamp negative thrusts to zero
    T_motors = max(T_motors, 0);

    % Convert thrust to motor speed: T = kT * omega^2 → omega = sqrt(T/kT)
    motor_cmds = sqrt(T_motors / kT);

    % Saturate motor speeds
    omega_min = dp.omega_min * 2 * pi / 60;
    omega_max = dp.omega_max * 2 * pi / 60;
    motor_cmds = max(omega_min, min(omega_max, motor_cmds));

end
