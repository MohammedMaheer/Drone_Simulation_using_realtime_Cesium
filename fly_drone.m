function fly_drone(mode)
% FLY_DRONE  Interactive launcher for the configurable drone simulator.
%
%   fly_drone            — Opens preset selector menu
%   fly_drone('quick')   — Instant launch with default quad
%   fly_drone('custom')  — Opens full custom configuration dialog
%   fly_drone('presets') — Shows preset browser
%
%   This is the main entry point for users. It provides a friendly
%   interface to drone_config + live_drone_sim.

    if nargin < 1
        mode = 'menu';
    end

    switch lower(mode)
        case 'quick'
            cfg = drone_config('standard_quad');
            live_drone_sim(cfg);

        case 'custom'
            cfg = custom_config_dialog();
            if ~isempty(cfg)
                live_drone_sim(cfg);
            end

        case 'presets'
            cfg = preset_browser();
            if ~isempty(cfg)
                live_drone_sim(cfg);
            end

        case 'menu'
            show_main_menu();

        otherwise
            % Try as preset name
            try
                cfg = drone_config(lower(mode));
                live_drone_sim(cfg);
            catch
                fprintf('Unknown mode: %s\n', mode);
                show_main_menu();
            end
    end
end


function show_main_menu()
    fprintf('\n');
    fprintf('  ===================================================\n');
    fprintf('  |         DRONE FLIGHT SIMULATOR LAUNCHER          |\n');
    fprintf('  ===================================================\n');
    fprintf('  |                                                   |\n');
    fprintf('  |  1. Quick Fly (450mm Quad)                       |\n');
    fprintf('  |  2. Preset Browser (5 drone types)               |\n');
    fprintf('  |  3. Custom Configuration (manual params)         |\n');
    fprintf('  |  4. Auto Configuration (smart derivation)        |\n');
    fprintf('  |  0. Cancel                                       |\n');
    fprintf('  |                                                   |\n');
    fprintf('  ===================================================\n\n');

    choice = input('  Select option [1-4]: ');

    switch choice
        case 1
            cfg = drone_config('standard_quad');
            live_drone_sim(cfg);
        case 2
            cfg = preset_browser();
            if ~isempty(cfg)
                live_drone_sim(cfg);
            end
        case 3
            cfg = custom_config_dialog();
            if ~isempty(cfg)
                live_drone_sim(cfg);
            end
        case 4
            cfg = auto_config_dialog();
            if ~isempty(cfg)
                live_drone_sim(cfg);
            end
        case 0
            fprintf('  Cancelled.\n');
        otherwise
            fprintf('  Invalid choice.\n');
    end
end


function cfg = preset_browser()
    presets = {
        'mini_quad',     '250mm Racing Quad',    '4 motors,  0.6 kg, 5" props, 4S, 2300Kv'
        'standard_quad', '450mm Standard Quad',  '4 motors,  1.5 kg, 10" props, 4S, 920Kv'
        'heavy_hex',     '680mm Heavy Hex',      '6 motors,  4.2 kg, 15" props, 6S, 580Kv'
        'octo_lift',     '1000mm Octocopter',    '8 motors,  8.0 kg, 18" props, 6S, 380Kv'
        'micro_tri',     '180mm Micro Tricopter','3 motors,  0.35 kg, 5" props, 3S, 2400Kv'
    };

    fprintf('\n');
    fprintf('  ===================================================\n');
    fprintf('  |              DRONE PRESET BROWSER                |\n');
    fprintf('  ===================================================\n');
    for i = 1:size(presets, 1)
        fprintf('  |  %d. %-38s |\n', i, presets{i,2});
        fprintf('  |     %s\n', presets{i,3});
        fprintf('  |                                                   |\n');
    end
    fprintf('  |  0. Back                                          |\n');
    fprintf('  ===================================================\n\n');

    choice = input('  Select preset [1-5]: ');

    if choice >= 1 && choice <= size(presets, 1)
        fprintf('\n  Loading %s...\n', presets{choice, 2});
        cfg = drone_config(presets{choice, 1});
    else
        cfg = [];
    end
end


function cfg = custom_config_dialog()
    fprintf('\n');
    fprintf('  ===================================================\n');
    fprintf('  |         CUSTOM DRONE CONFIGURATION               |\n');
    fprintf('  |         (Manual Mode - You set everything)       |\n');
    fprintf('  ===================================================\n\n');

    fprintf('  --- FRAME ---\n');
    fprintf('  Frame types: tri, quad_x, quad_+, hex_flat, hex_y, octo_flat, octo_x\n');
    frame = input('  Frame type [quad_x]: ', 's');
    if isempty(frame); frame = 'quad_x'; end

    n_map = struct('tri',3, 'quad_x',4, 'quad_',4, 'hex_flat',6, ...
                   'hex_y',6, 'octo_flat',8, 'octo_x',8);
    if isfield(n_map, strrep(frame,'+','_'))
        n_motors = n_map.(strrep(frame,'+','_'));
    else
        n_motors = input('  Number of motors [4]: ');
        if isempty(n_motors); n_motors = 4; end
    end

    arm = input('  Arm length (mm) [230]: ');
    if isempty(arm); arm = 230; end

    mass = input('  Total mass (kg) [1.5]: ');
    if isempty(mass); mass = 1.5; end

    fprintf('\n  --- PROPULSION ---\n');
    prop_d = input('  Propeller diameter (inches) [10]: ');
    if isempty(prop_d); prop_d = 10; end

    prop_p = input('  Propeller pitch (inches) [4.5]: ');
    if isempty(prop_p); prop_p = 4.5; end

    kv = input('  Motor Kv [920]: ');
    if isempty(kv); kv = 920; end

    max_rpm = input('  Motor max RPM [8000]: ');
    if isempty(max_rpm); max_rpm = 8000; end

    kt = input('  Thrust coefficient kT [1.5e-5] (0=auto): ');
    if isempty(kt) || kt == 0; kt = 1.5e-5; end

    kq = input('  Torque coefficient kQ [2.5e-7] (0=auto): ');
    if isempty(kq) || kq == 0; kq = 2.5e-7; end

    fprintf('\n  --- BATTERY ---\n');
    fprintf('  Types: 2S, 3S, 4S, 5S, 6S\n');
    batt_type = input('  Battery type [4S]: ', 's');
    if isempty(batt_type); batt_type = '4S'; end

    batt_cap = input('  Battery capacity (Ah) [5.0]: ');
    if isempty(batt_cap); batt_cap = 5.0; end

    batt_c = input('  Discharge C rating [25]: ');
    if isempty(batt_c); batt_c = 25; end

    fprintf('\n  --- INERTIA (kg*m^2, 0 = auto-estimate) ---\n');
    ixx = input('  Ixx [0]: ');
    if isempty(ixx) || ixx == 0; ixx = 0.0135; end
    iyy = input('  Iyy [0]: ');
    if isempty(iyy) || iyy == 0; iyy = 0.0135; end
    izz = input('  Izz [0]: ');
    if isempty(izz) || izz == 0; izz = 0.024; end

    fprintf('\n  Building configuration...\n');

    cfg = drone_config('manual', ...
        'NumMotors', n_motors, ...
        'FrameType', frame, ...
        'ArmLength', arm / 1000, ...
        'Mass', mass, ...
        'PropDiameter', prop_d * 0.0254, ...
        'PropPitch', prop_p * 0.0254, ...
        'MotorKv', kv, ...
        'MotorMaxRPM', max_rpm, ...
        'kT', kt, ...
        'kQ', kq, ...
        'BatteryType', batt_type, ...
        'BatteryCapacity', batt_cap, ...
        'BatteryC', batt_c, ...
        'Ixx', ixx, ...
        'Iyy', iyy, ...
        'Izz', izz);

    fprintf('\n  Ready to fly? (y/n): ');
    ans_str = input('', 's');
    if ~strcmpi(ans_str, 'y')
        cfg = [];
    end
end


function cfg = auto_config_dialog()
    fprintf('\n');
    fprintf('  ===================================================\n');
    fprintf('  |         AUTO DRONE CONFIGURATION                 |\n');
    fprintf('  |  (Smart Mode - Give basics, we derive the rest)  |\n');
    fprintf('  ===================================================\n\n');
    fprintf('  Just provide the basic specs. The system will:\n');
    fprintf('  - Calculate kT, kQ from propeller geometry\n');
    fprintf('  - Estimate inertia from frame dimensions\n');
    fprintf('  - Derive motor limits from Kv + battery\n');
    fprintf('  - Auto-tune PID controller gains\n');
    fprintf('  - Estimate flight time and performance\n\n');

    fprintf('  --- FRAME ---\n');
    fprintf('  Frame types: tri, quad_x, quad_+, hex_flat, hex_y, octo_flat, octo_x\n');
    frame = input('  Frame type [quad_x]: ', 's');
    if isempty(frame); frame = 'quad_x'; end

    n_map = struct('tri',3, 'quad_x',4, 'quad_',4, 'hex_flat',6, ...
                   'hex_y',6, 'octo_flat',8, 'octo_x',8);
    if isfield(n_map, strrep(frame,'+','_'))
        n_motors = n_map.(strrep(frame,'+','_'));
    else
        n_motors = input('  Number of motors [4]: ');
        if isempty(n_motors); n_motors = 4; end
    end

    arm = input('  Arm length (mm) [230]: ');
    if isempty(arm); arm = 230; end

    mass = input('  Total mass (kg) [1.5]: ');
    if isempty(mass); mass = 1.5; end

    fprintf('\n  --- PROPULSION ---\n');
    prop_d = input('  Propeller diameter (inches) [10]: ');
    if isempty(prop_d); prop_d = 10; end

    prop_p = input('  Propeller pitch (inches) [4.5]: ');
    if isempty(prop_p); prop_p = 4.5; end

    kv = input('  Motor Kv [920]: ');
    if isempty(kv); kv = 920; end

    fprintf('\n  --- BATTERY ---\n');
    batt_type = input('  Battery type (2S-6S) [4S]: ', 's');
    if isempty(batt_type); batt_type = '4S'; end

    batt_cap = input('  Battery capacity (Ah) [5.0]: ');
    if isempty(batt_cap); batt_cap = 5.0; end

    fprintf('\n  Auto-deriving all parameters...\n');

    cfg = drone_config('auto', ...
        'NumMotors', n_motors, ...
        'FrameType', frame, ...
        'ArmLength', arm / 1000, ...
        'Mass', mass, ...
        'PropDiameter', prop_d * 0.0254, ...
        'PropPitch', prop_p * 0.0254, ...
        'MotorKv', kv, ...
        'BatteryType', batt_type, ...
        'BatteryCapacity', batt_cap);

    fprintf('\n  Ready to fly? (y/n): ');
    ans_str = input('', 's');
    if ~strcmpi(ans_str, 'y')
        cfg = [];
    end
end
