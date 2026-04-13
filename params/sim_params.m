function params = sim_params()
% SIM_PARAMS  Simulation configuration parameters.
%
%   params = sim_params() returns a struct with timing, environment,
%   solver, and scenario settings for the drone simulation.

    %% ===== Timing =====
    params.t_start  = 0;        % Simulation start time [s]
    params.t_end    = 60;       % Simulation end time [s]
    params.dt       = 0.001;    % Fixed time step [s] (1 kHz)
    params.dt_viz   = 0.02;     % Visualization update interval [s] (50 Hz)
    params.dt_log   = 0.01;     % Telemetry logging interval [s] (100 Hz)

    %% ===== Solver =====
    params.solver       = 'ode4';     % Simulink solver ('ode4' = RK4 fixed-step)
    params.solver_type  = 'fixed';    % 'fixed' or 'variable'

    %% ===== Initial Conditions =====
    params.init.position    = [0; 0; 0];       % [x; y; z] NED [m] (z=0 is ground)
    params.init.velocity    = [0; 0; 0];       % [vx; vy; vz] [m/s]
    params.init.euler       = [0; 0; 0];       % [roll; pitch; yaw] [rad]
    params.init.omega       = [0; 0; 0];       % [p; q; r] body angular rates [rad/s]
    params.init.motor_speed = [0; 0; 0; 0];    % Motor speeds [rad/s]

    %% ===== Environment =====
    params.env.wind_enabled    = true;
    params.env.wind_mean       = [1.0; 0.5; 0];      % Mean wind velocity NED [m/s]
    params.env.wind_gust_amp   = [2.0; 2.0; 0.5];    % Gust amplitude [m/s]
    params.env.wind_gust_freq  = [0.5; 0.3; 0.2];    % Gust frequency [Hz]
    params.env.temperature     = 20;                   % Ambient temperature [°C]
    params.env.pressure_sea    = 101325;               % Sea-level pressure [Pa]

    %% ===== Scenario Defaults =====
    params.scenario.name           = 'hover';
    params.scenario.target_alt     = 10;       % Default target altitude [m]
    params.scenario.takeoff_speed  = 1.5;      % Takeoff climb rate [m/s]
    params.scenario.landing_speed  = 0.8;      % Landing descent rate [m/s]
    params.scenario.geofence_radius = 100;     % Geofence radius [m]
    params.scenario.geofence_height = 120;     % Max altitude [m]

    %% ===== Logging =====
    params.log.enabled         = true;
    params.log.save_to_file    = true;
    params.log.filename        = 'flight_log.mat';
    params.log.variables       = {'time','position','velocity','euler',...
                                  'omega','motor_speeds','thrust',...
                                  'control_cmd','battery_soc'};

    %% ===== Visualization =====
    params.viz.enabled         = true;
    params.viz.realtime_plot   = true;
    params.viz.trail_length    = 500;       % Number of trail points
    params.viz.figure_size     = [100, 100, 1200, 700];

end
