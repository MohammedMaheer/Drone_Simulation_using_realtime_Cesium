function params = drone_params()
% DRONE_PARAMS  Physical parameters for a generic quadrotor UAV.
%
%   params = drone_params() returns a struct with mass, inertia, geometry,
%   motor, propeller, aerodynamic, and environmental constants.

    %% Mass & Inertia
    params.mass   = 1.5;       % Total mass [kg]
    params.Ixx    = 0.0135;    % Moment of inertia about x-axis [kg·m^2]
    params.Iyy    = 0.0135;    % Moment of inertia about y-axis [kg·m^2]
    params.Izz    = 0.0240;    % Moment of inertia about z-axis [kg·m^2]
    params.I      = diag([params.Ixx, params.Iyy, params.Izz]);

    %% Geometry
    params.arm_length = 0.23;  % Center-to-motor distance [m]
    params.d_prop     = 0.254; % Propeller diameter [m] (10 inch)

    %% Motor Constants (brushless DC)
    params.Kv         = 920;       % Motor Kv rating [RPM/V]
    params.tau_motor   = 0.02;     % Motor time constant [s]
    params.omega_max   = 8000;     % Maximum motor speed [RPM]
    params.omega_min   = 1200;     % Minimum motor speed [RPM]

    %% Thrust & Torque Coefficients
    params.kT = 1.5e-5;   % Thrust coefficient: T = kT * omega^2 [N/(rad/s)^2]
    params.kQ = 2.5e-7;   % Torque coefficient: Q = kQ * omega^2 [N·m/(rad/s)^2]

    %% Aerodynamic Drag
    params.Cd_xy = 0.25;  % Translational drag coefficient (horizontal)
    params.Cd_z  = 0.50;  % Translational drag coefficient (vertical)
    params.Cd_r  = 0.01;  % Rotational drag coefficient

    %% Ground Effect
    params.ground_effect_height = 0.5;  % Height below which ground effect is active [m]
    params.ground_effect_gain   = 0.1;  % Ground effect thrust multiplier

    %% Battery
    params.V_battery   = 14.8;   % Nominal battery voltage [V] (4S LiPo)
    params.capacity_Ah = 5.0;    % Battery capacity [Ah]

    %% Environment
    params.g       = 9.81;      % Gravitational acceleration [m/s^2]
    params.rho_air = 1.225;     % Air density at sea level [kg/m^3]

    %% Derived Quantities
    params.hover_omega = sqrt(params.mass * params.g / (4 * params.kT));  % Hover speed [rad/s]
    params.hover_thrust_per_motor = params.mass * params.g / 4;           % Hover thrust [N]

end
