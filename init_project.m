function init_project()
% INIT_PROJECT  Initialize the drone simulation project.
%
%   Adds all project subdirectories to the MATLAB path and displays
%   a welcome message with available commands.
%
%   Usage: Run this script first after opening MATLAB in the project folder.

    %% Get project root
    project_root = fileparts(mfilename('fullpath'));

    %% Add subdirectories to path
    folders = {'params', 'models', 'control', 'navigation', ...
               'sensors', 'telemetry', 'visualization', 'tests', 'utilities', ...
               'environment', 'tests/unit', 'tests/integration'};

    for i = 1:length(folders)
        folder_path = fullfile(project_root, folders{i});
        if exist(folder_path, 'dir')
            addpath(folder_path);
        end
    end

    % Add root
    addpath(project_root);

    %% Display welcome
    fprintf('\n');
    fprintf('╔══════════════════════════════════════════════════════════╗\n');
    fprintf('║        MULTIROTOR DRONE SIMULATION — INITIALIZED        ║\n');
    fprintf('╠══════════════════════════════════════════════════════════╣\n');
    fprintf('║                                                          ║\n');
    fprintf('║  === FLY (new configurable simulator) ===                ║\n');
    fprintf('║    fly_drone              — Interactive launcher menu     ║\n');
    fprintf('║    fly_drone(''quick'')     — Instant flight (std quad)   ║\n');
    fprintf('║    fly_drone(''custom'')    — Full manual config dialog   ║\n');
    fprintf('║    fly_drone(''presets'')   — Browse 5 drone presets      ║\n');
    fprintf('║    fly_drone(''mini_quad'') — Direct preset launch        ║\n');
    fprintf('║                                                          ║\n');
    fprintf('║  === Advanced Config ===                                 ║\n');
    fprintf('║    cfg = drone_config(''standard_quad'')                  ║\n');
    fprintf('║    cfg = drone_config(''auto'', ''Mass'',2, ...)            ║\n');
    fprintf('║    cfg = drone_config(''manual'', ''NumMotors'',6, ...)     ║\n');
    fprintf('║    live_drone_sim(cfg)    — Launch with custom config    ║\n');
    fprintf('║                                                          ║\n');
    fprintf('║  === Classic Simulation ===                              ║\n');
    fprintf('║    run_simulation         — Run default simulation       ║\n');
    fprintf('║    run_simulation(''circle'') — Fly circle mission        ║\n');
    fprintf('║    build_drone_simulink   — Build Simulink model         ║\n');
    fprintf('║                                                          ║\n');
    fprintf('║  === Tests ===                                           ║\n');
    fprintf('║    run_all_tests          — Full test suite              ║\n');
    fprintf('║    test_hover | test_waypoint_nav | test_wind_disturbance║\n');
    fprintf('║    test_sensor_failure                                   ║\n');
    fprintf('║                                                          ║\n');
    fprintf('║  Presets: mini_quad, standard_quad, heavy_hex,           ║\n');
    fprintf('║           octo_lift, micro_tri                           ║\n');
    fprintf('║  Frames:  tri, quad_x, quad_+, hex_flat, hex_y,         ║\n');
    fprintf('║           octo_flat, octo_x                              ║\n');
    fprintf('║                                                          ║\n');
    fprintf('║  === Physics Features ===                                ║\n');
    fprintf('║    Dryden wind turbulence (MIL-DTL-9490E)               ║\n');
    fprintf('║    Battery thermal model with cutoff protection          ║\n');
    fprintf('║    Propeller vibration (1/rev + 2/rev + resonance)      ║\n');
    fprintf('║    Sensor latency & dropout simulation                   ║\n');
    fprintf('║    Cheeseman-Bennett ground effect                       ║\n');
    fprintf('║                                                          ║\n');
    fprintf('║  In-flight keys: WASD=move, QE=yaw, Space/Shift=alt     ║\n');
    fprintf('║    M=auto/manual, H=hover, TAB=HUD page, 4=battery      ║\n');
    fprintf('║                                                          ║\n');
    fprintf('╚══════════════════════════════════════════════════════════╝\n');
    fprintf('\n');

end
