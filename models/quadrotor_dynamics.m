function [state_dot, forces, moments] = quadrotor_dynamics(state, motor_speeds, wind, dp)
% QUADROTOR_DYNAMICS  Full 6-DOF rigid-body quadrotor dynamics.
%
%   [state_dot, forces, moments] = quadrotor_dynamics(state, motor_speeds, wind, dp)
%
%   Inputs:
%     state        - [12x1] state vector: [x y z vx vy vz phi theta psi p q r]'
%     motor_speeds - [4x1] motor angular velocities [rad/s]
%     wind         - [3x1] wind velocity in NED frame [m/s]
%     dp           - drone_params struct
%
%   Outputs:
%     state_dot - [12x1] time derivative of state
%     forces    - [3x1] total forces in NED frame [N]
%     moments   - [3x1] total moments in body frame [N·m]
%
%   State Convention:
%     NED frame: x=North, y=East, z=Down (positive altitude = negative z)
%     Body frame: x=Forward, y=Right, z=Down
%     Motor order: [Front-Right, Rear-Left, Front-Left, Rear-Right]
%                  Motors 1,2 spin CW; Motors 3,4 spin CCW

    %% Unpack State
    % Position (NED)
    pos = state(1:3);
    % Velocity (NED)
    vel = state(4:6);
    % Euler angles (roll, pitch, yaw)
    phi   = state(7);   % Roll
    theta = state(8);   % Pitch
    psi   = state(9);   % Yaw
    % Body angular rates
    omega_b = state(10:12); % [p; q; r]
    p = omega_b(1);
    q = omega_b(2);
    r = omega_b(3);

    %% Motor Forces & Torques
    % Thrust per motor: T_i = kT * omega_i^2
    T = dp.kT * motor_speeds.^2;        % [4x1] thrust per motor [N]
    Q = dp.kQ * motor_speeds.^2;        % [4x1] reaction torque per motor [N·m]

    % Total thrust (along body z-axis, pointing up = negative body-z)
    total_thrust = sum(T);

    % Moments from motors
    L = dp.arm_length;
    % Standard X-configuration mixing:
    %   Motor 1: Front-Right (CW)   (+L/sqrt(2), +L/sqrt(2))
    %   Motor 2: Rear-Left   (CW)   (-L/sqrt(2), -L/sqrt(2))
    %   Motor 3: Front-Left  (CCW)  (+L/sqrt(2), -L/sqrt(2))
    %   Motor 4: Rear-Right  (CCW)  (-L/sqrt(2), +L/sqrt(2))
    s45 = sqrt(2) / 2;
    tau_x = L * s45 * (-T(1) + T(2) + T(3) - T(4));   % Roll moment (y-position)
    tau_y = L * s45 * ( T(1) - T(2) + T(3) - T(4));    % Pitch moment (x-position)
    tau_z = -Q(1) - Q(2) + Q(3) + Q(4);                % Yaw moment (reaction torques)

    moments_motors = [tau_x; tau_y; tau_z];

    %% Rotation Matrix (Body → NED)
    R = rotation_matrix_euler(phi, theta, psi);

    %% Gravity Force (NED)
    F_gravity = [0; 0; dp.mass * dp.g];

    %% Thrust Force (NED)
    % Thrust acts along negative body z-axis
    F_thrust_body = [0; 0; -total_thrust];
    F_thrust_ned  = R * F_thrust_body;

    %% Aerodynamic Drag (NED)
    vel_air = vel - wind;   % Airspeed in NED frame
    F_drag_ned = -[dp.Cd_xy; dp.Cd_xy; dp.Cd_z] .* vel_air .* abs(vel_air);

    %% Ground Effect (Cheeseman-Bennett, 1955)
    % T_ge/T_oge = 1 / (1 - (R/(4h))^2)
    altitude = -pos(3);  % Convert NED z to altitude
    if altitude < dp.ground_effect_height && altitude > 0.05
        rotor_R = dp.d_prop / 2;
        ratio = rotor_R / (4 * altitude);
        if ratio < 1
            ge_factor = 1.0 / (1.0 - ratio^2);
            ge_factor = min(ge_factor, 1.5);
        else
            ge_factor = 1.5;
        end
        F_thrust_ned = F_thrust_ned * ge_factor;
    end

    %% Total Forces (NED)
    forces = F_gravity + F_thrust_ned + F_drag_ned;

    %% Ground Contact
    if pos(3) >= 0 && forces(3) > 0
        forces(3) = 0;       % No penetration into ground
        if vel(3) > 0
            vel(3) = 0;      % Kill downward velocity
        end
    end

    %% Rotational Drag
    moments_drag = -dp.Cd_r * omega_b .* abs(omega_b);

    %% Total Moments (Body frame)
    moments = moments_motors + moments_drag;

    %% Translational Dynamics (NED frame)
    pos_dot = vel;
    vel_dot = forces / dp.mass;

    %% Euler Angle Kinematics
    % Angular velocity to Euler rate transformation
    euler_dot = euler_rate_matrix(phi, theta) * omega_b;

    %% Rotational Dynamics (Body frame, Euler's equation)
    I  = dp.I;
    omega_dot = I \ (moments - cross(omega_b, I * omega_b));

    %% Pack State Derivative
    state_dot = [pos_dot; vel_dot; euler_dot; omega_dot];

end


function R = rotation_matrix_euler(phi, theta, psi)
% ZYX Euler rotation matrix (Body → NED)
    cphi = cos(phi);   sphi = sin(phi);
    cth  = cos(theta); sth  = sin(theta);
    cpsi = cos(psi);   spsi = sin(psi);

    R = [cth*cpsi,  sphi*sth*cpsi - cphi*spsi,  cphi*sth*cpsi + sphi*spsi;
         cth*spsi,  sphi*sth*spsi + cphi*cpsi,  cphi*sth*spsi - sphi*cpsi;
         -sth,      sphi*cth,                    cphi*cth                  ];
end


function E = euler_rate_matrix(phi, theta)
% Maps body angular rates [p;q;r] → Euler rates [phi_dot; theta_dot; psi_dot]
    % Gimbal lock protection: clamp theta away from ±90°
    max_pitch = deg2rad(80);
    theta = max(-max_pitch, min(max_pitch, theta));

    cphi = cos(phi);
    sphi = sin(phi);
    cth  = cos(theta);
    tth  = tan(theta);

    E = [1,  sphi*tth,  cphi*tth;
         0,  cphi,      -sphi;
         0,  sphi/cth,  cphi/cth];
end
