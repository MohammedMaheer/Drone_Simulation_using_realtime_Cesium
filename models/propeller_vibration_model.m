function [vib_forces, vib_moments] = propeller_vibration_model(motor_speeds, ...
        motor_layout, dp, t)
% PROPELLER_VIBRATION_MODEL  Simulates propeller imbalance and structural vibration.
%
%   [vib_forces, vib_moments] = propeller_vibration_model(motor_speeds, ...
%       motor_layout, dp, t)
%
%   Models three vibration sources:
%     1) Static imbalance: 1/rev vibration from prop mass asymmetry (dominant)
%     2) Dynamic imbalance: 2/rev from blade track difference
%     3) Structural resonance: amplification near natural frequency
%
%   Inputs:
%     motor_speeds - [Nx1] motor angular velocities [rad/s]
%     motor_layout - struct with .positions [Nx2] and .spin_dirs [Nx1]
%     dp           - drone config struct
%     t            - simulation time [s]
%
%   Outputs:
%     vib_forces  - [3x1] vibration forces in body frame [N]
%     vib_moments - [3x1] vibration moments in body frame [N·m]
%
%   Reference:
%     Prassad et al. (2015), "Analysis of helicopter vibration"
%     Pounds et al. (2010), "Modelling and control of a large quad-rotor"

    n = length(motor_speeds);

    %% Imbalance parameters
    % Static imbalance: equivalent mass offset from center
    % Typical: 0.1-0.5% of prop mass at tip radius
    if isfield(dp, 'prop_imbalance')
        imbalance = dp.prop_imbalance;  % [0..1] 0=perfect, 1=severe
    else
        imbalance = 0.02;  % Default: 2% — excellent prop balance
    end

    r_prop = dp.d_prop / 2;
    m_prop_est = 0.00008 * (dp.d_prop / 0.0254)^2.3;  % Prop mass estimate

    % Equivalent unbalanced mass * radius [kg·m]
    m_e_r = imbalance * m_prop_est * r_prop * 0.5;

    %% Compute per-motor vibration
    vib_forces  = [0; 0; 0];
    vib_moments = [0; 0; 0];

    for i = 1:n
        omega_i = motor_speeds(i);
        freq_i = omega_i / (2*pi);  % Hz

        % Phase angle — each prop has random initial phase (set per-sim)
        % Use motor index for deterministic, repeatable vibration
        phase_i = 2 * pi * i / n;

        %% 1/rev — static imbalance (centrifugal force)
        % F = m_e * r * omega^2   (rotating in plane of prop disc)
        F_1rev = m_e_r * omega_i^2;
        % This force rotates at omega_i; project into body x,y
        angle_1rev = omega_i * t + phase_i;
        Fx_1rev = F_1rev * cos(angle_1rev);
        Fy_1rev = F_1rev * sin(angle_1rev);
        Fz_1rev = 0;  % In-plane force

        %% 2/rev — dynamic imbalance (blade track split)
        % Much smaller: ~10% of 1/rev amplitude
        F_2rev = 0.1 * F_1rev;
        angle_2rev = 2 * omega_i * t + phase_i;
        Fz_2rev = F_2rev * cos(angle_2rev);  % Vertical (thrust axis)

        %% Motor position contribution to moments
        xi = motor_layout.positions(i, 1);
        yi = motor_layout.positions(i, 2);

        % Accumulate forces
        vib_forces = vib_forces + [Fx_1rev; Fy_1rev; Fz_2rev];

        % Moments from vibration forces at motor positions
        vib_moments(1) = vib_moments(1) - yi * Fz_2rev;  % Roll
        vib_moments(2) = vib_moments(2) + xi * Fz_2rev;  % Pitch
        vib_moments(3) = vib_moments(3) + (-yi * Fx_1rev + xi * Fy_1rev);  % Yaw
    end

    %% Structural resonance amplification (optional)
    % If any motor frequency is near the frame natural frequency,
    % amplify vibrations (Q-factor of ~5 is typical for carbon fiber frames)
    if isfield(dp, 'frame_nat_freq')
        f_nat = dp.frame_nat_freq;  % Hz
    else
        % Estimate from arm length: f ≈ 100/L^0.7 Hz (empirical)
        f_nat = 100 / dp.arm_length^0.7;
    end
    Q_factor = 5;  % Damping quality factor

    for i = 1:n
        freq_i = motor_speeds(i) / (2*pi);
        % Frequency ratio
        r_f = freq_i / f_nat;
        if r_f > 0.3 && r_f < 3.0
            % Transmissibility: T = 1/sqrt((1-r²)² + (2*zeta*r)²)
            zeta = 1 / (2 * Q_factor);
            T = 1 / sqrt((1 - r_f^2)^2 + (2*zeta*r_f)^2);
            T = min(T, Q_factor);  % Cap at Q
            if T > 1.5
                amplification = (T - 1) / (n * Q_factor);
                vib_forces = vib_forces * (1 + amplification);
                vib_moments = vib_moments * (1 + amplification);
            end
        end
    end

end
