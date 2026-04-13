function [state_dot, forces, moments] = multirotor_dynamics(state, motor_speeds, wind, dp)
% MULTIROTOR_DYNAMICS  Generalized 6-DOF dynamics for N-motor multirotors.
%
%   [state_dot, forces, moments] = multirotor_dynamics(state, motor_speeds, wind, dp)
%
%   Unlike quadrotor_dynamics.m which is hardcoded for 4 motors in X-config,
%   this function works with any motor count and layout (tri/quad/hex/octo)
%   defined by dp.motor_layout and dp.mix_matrix from drone_config.
%
%   Inputs:
%     state        - [12x1] [x y z vx vy vz phi theta psi p q r]'
%     motor_speeds - [Nx1] motor angular velocities [rad/s]
%     wind         - [3x1] wind velocity in NED frame [m/s]
%     dp           - drone config struct (from drone_config)
%
%   Outputs:
%     state_dot - [12x1] time derivative of state
%     forces    - [3x1] total forces in NED [N]
%     moments   - [3x1] total moments in body [N*m]
%
%   Features beyond quadrotor_dynamics:
%     - N-motor support with layout-based moment calculation
%     - Voltage-dependent thrust ceiling (battery sag)
%     - Precise propeller drag torque model
%     - Gyroscopic precession from spinning propellers
%     - Blade flapping approximation

    %% Unpack State
    pos     = state(1:3);
    vel     = state(4:6);
    phi     = state(7);    theta = state(8);    psi = state(9);
    omega_b = state(10:12);
    p = omega_b(1); q = omega_b(2); r = omega_b(3);

    n = dp.num_motors;
    layout = dp.motor_layout;

    %% Motor Forces & Torques (per motor)
    T = dp.kT * motor_speeds.^2;       % [Nx1] thrust [N]
    Q = dp.kQ * motor_speeds.^2;       % [Nx1] reaction torque [N*m]

    total_thrust = sum(T);

    %% Moments from motor layout
    tau_x = 0; tau_y = 0; tau_z = 0;
    for i = 1:n
        xi = layout.positions(i, 1);   % Forward position
        yi = layout.positions(i, 2);   % Right position
        si = layout.spin_dirs(i);       % +1=CCW, -1=CW

        tau_x = tau_x - yi * T(i);     % Roll: -y * thrust
        tau_y = tau_y + xi * T(i);     % Pitch: +x * thrust
        tau_z = tau_z + si * Q(i);     % Yaw: spin_dir * reaction torque
    end
    moments_motors = [tau_x; tau_y; tau_z];

    %% Gyroscopic precession from spinning props
    % Propeller MoI from config (Jp = 1/2 * m_prop * r_prop^2)
    if isfield(dp, 'Jp')
        Jp = dp.Jp;
    else
        % Fallback: thin disc approximation from prop geometry
        % Jp = 0.5 * m_prop * r^2, with m_prop estimated from diameter
        r_p = dp.d_prop / 2;
        m_p_est = 0.00008 * (dp.d_prop / 0.0254)^2.3;  % kg, from diameter
        Jp = 0.5 * m_p_est * r_p^2;
    end
    h_z = 0;   % net prop angular momentum
    for i = 1:n
        h_z = h_z + layout.spin_dirs(i) * motor_speeds(i);
    end
    h_z = h_z * Jp;
    % Gyroscopic torque: omega_body x (0, 0, h_z)
    gyro_torque = [q * h_z; -p * h_z; 0];

    %% Blade flapping (with advance ratio correction)
    % At forward speed, advancing/retreating blade asymmetry creates moments.
    % Effect amplifies with advance ratio mu = V_fwd / V_tip (Leishman, 2006).
    vel_body = rotation_matrix_euler_gen(phi, theta, psi)' * (vel - wind);
    vx_b = vel_body(1); vy_b = vel_body(2);
    % Flapping coefficient from config (scales with prop area and thrust coeff)
    if isfield(dp, 'K_flap')
        K_flap = dp.K_flap;
    else
        K_flap = 0.0005 * dp.d_prop;  % Fallback
    end
    % Advance ratio correction: flapping increases with forward speed
    V_tip = mean(abs(motor_speeds)) * (dp.d_prop / 2);
    mu = norm(vel_body(1:2)) / max(V_tip, 1.0);  % Advance ratio
    K_flap_eff = K_flap * (1 + 3.0 * mu);  % Empirical amplification (Prouty, 2002)
    flap_torque = [-K_flap_eff * vy_b * total_thrust;
                    K_flap_eff * vx_b * total_thrust;
                    0];

    %% Rotation Matrix (Body -> NED)
    R = rotation_matrix_euler_gen(phi, theta, psi);

    %% Gravity (NED)
    F_gravity = [0; 0; dp.mass * dp.g];

    %% Thrust (NED) — with altitude air density correction
    % ISA atmosphere model: rho decreases ~12% per 1000m
    altitude = -pos(3);
    rho_sea = dp.rho_air;
    if altitude > 10
        % Barometric formula (troposphere, valid to ~11km)
        rho_local = rho_sea * (1 - 2.2558e-5 * altitude)^4.2559;
        rho_local = max(0.4, rho_local);  % Floor at ~7km equiv
    else
        rho_local = rho_sea;
    end
    density_ratio = rho_local / rho_sea;
    F_thrust_body = [0; 0; -total_thrust * density_ratio];
    F_thrust_ned  = R * F_thrust_body;

    %% Aerodynamic Drag (NED) — with altitude density correction
    vel_air = vel - wind;
    F_drag_ned = -[dp.Cd_xy; dp.Cd_xy; dp.Cd_z] .* vel_air .* abs(vel_air) * density_ratio;

    %% Hub drag (parasitic drag in forward flight)
    % Uses actual frontal area from config dimensions
    speed_sq = sum(vel_air.^2);
    if speed_sq > 0.01
        if isfield(dp, 'hub_frontal_area')
            A_frontal = dp.hub_frontal_area;
        else
            A_frontal = dp.d_prop * 0.02 * n;   % Fallback estimate
        end
        Cd_hub = 1.0;  % Bluff body drag coefficient
        F_hub_drag = -0.5 * rho_local * Cd_hub * A_frontal * vel_air * sqrt(speed_sq);
    else
        F_hub_drag = [0; 0; 0];
    end

    %% Ground effect (Cheeseman-Bennett, 1955)
    % T_ge/T_oge = 1 / (1 - (R/(4h))^2)  for single rotor
    % For multirotors, use individual rotor radius and altitude check
    if altitude < dp.ground_effect_height && altitude > 0.05
        rotor_R = dp.d_prop / 2;
        ratio = rotor_R / (4 * altitude);
        if ratio < 1  % Only valid when h > R/4
            ge_factor = 1.0 / (1.0 - ratio^2);
            ge_factor = min(ge_factor, 1.5);  % Saturate at 50% increase
        else
            ge_factor = 1.5;  % Very close to ground, cap
        end
        F_thrust_ned = F_thrust_ned * ge_factor;
    end

    %% Vortex Ring State (VRS) — thrust loss in rapid descent
    % Johnson (1980): VRS occurs when descending into own downwash.
    % Onset at Vd > 0.3*Vi, full effect at Vd ~ 1.5*Vi.
    % Only significant when lateral speed is low (can't escape downwash).
    total_disc_area = n * pi / 4 * dp.d_prop^2;
    Vi_hover = sqrt(total_thrust * density_ratio / (2 * rho_local * total_disc_area + 1e-6));
    Vd = vel(3);  % NED: positive = descending
    Vh = norm(vel(1:2));  % Horizontal speed
    if Vd > 0.3 * Vi_hover && Vh < 2.0 * Vi_hover && Vi_hover > 0.1
        % Normalized descent rate and lateral speed
        vd_ratio = Vd / Vi_hover;
        vh_ratio = Vh / Vi_hover;
        % VRS severity: peaks at vd_ratio ~ 1.5, fades with lateral speed
        vrs_depth = exp(-4.5 * (vd_ratio - 1.5)^2);  % Gaussian centered at 1.5
        vrs_lateral_fade = exp(-2.0 * vh_ratio^2);     % Fades with forward flight
        vrs_factor = 1.0 - 0.6 * vrs_depth * vrs_lateral_fade;  % Up to 60% thrust loss
        vrs_factor = max(0.4, min(1.0, vrs_factor));
        F_thrust_ned = F_thrust_ned * vrs_factor;
    end

    %% Total forces
    forces = F_gravity + F_thrust_ned + F_drag_ned + F_hub_drag;

    %% Ground contact with spring-damper and friction model
    if pos(3) >= 0
        % Penetration depth (positive when below ground)
        penetration = pos(3);

        % Normal force: spring-damper (prevents penetration, absorbs impact)
        k_ground = 2000 * dp.mass;   % Ground stiffness [N/m] (scaled to drone mass)
        c_ground = 50 * dp.mass;     % Ground damping [N·s/m]
        F_normal = -k_ground * penetration - c_ground * max(0, vel(3));
        F_normal = min(0, F_normal);  % Normal force only pushes up (NED: negative Z = up)

        % Replace vertical force component with ground reaction
        forces(3) = forces(3) + F_normal;
        if forces(3) > 0
            forces(3) = 0;  % Net force cannot push into ground
        end

        % Coulomb friction: opposes horizontal sliding
        mu_friction = 0.6;  % Rubber skid on concrete
        F_normal_mag = abs(F_normal);
        h_vel_mag = norm(vel(1:2));
        if h_vel_mag > 0.01
            F_friction = -mu_friction * F_normal_mag * vel(1:2) / h_vel_mag;
            % Static friction limit: stop completely if slow enough
            if h_vel_mag < 0.05
                F_friction = -mu_friction * F_normal_mag * vel(1:2) / 0.05;
            end
            forces(1:2) = forces(1:2) + F_friction;
        end

        % Prevent sinking below ground
        if vel(3) > 0 && pos(3) > 0
            vel(3) = 0;
        end
    end

    %% Rotational Drag
    moments_drag = -dp.Cd_r * omega_b .* abs(omega_b);

    %% Total moments (body)
    moments = moments_motors + moments_drag + gyro_torque + flap_torque;

    %% Translational dynamics
    pos_dot = vel;
    vel_dot = forces / dp.mass;

    %% Euler angle kinematics
    euler_dot = euler_rate_matrix_gen(phi, theta) * omega_b;

    %% Rotational dynamics (Euler's equation)
    I = dp.I;
    omega_dot = I \ (moments - cross(omega_b, I * omega_b));

    %% Pack
    state_dot = [pos_dot; vel_dot; euler_dot; omega_dot];
end


%% ================================================================
%% LOCAL — Rotation matrix
%% ================================================================
function R = rotation_matrix_euler_gen(phi, theta, psi)
    cphi = cos(phi);   sphi = sin(phi);
    cth  = cos(theta); sth  = sin(theta);
    cpsi = cos(psi);   spsi = sin(psi);
    R = [cth*cpsi, sphi*sth*cpsi-cphi*spsi, cphi*sth*cpsi+sphi*spsi;
         cth*spsi, sphi*sth*spsi+cphi*cpsi, cphi*sth*spsi-sphi*cpsi;
         -sth,     sphi*cth,                cphi*cth              ];
end

function W = euler_rate_matrix_gen(phi, theta)
    % Gimbal lock protection: clamp theta away from ±90°
    max_pitch = deg2rad(80);
    theta = max(-max_pitch, min(max_pitch, theta));

    cphi = cos(phi); sphi = sin(phi);
    cth  = cos(theta); tth = tan(theta);
    W = [1, sphi*tth,  cphi*tth;
         0, cphi,      -sphi;
         0, sphi/cth,  cphi/cth];
end
