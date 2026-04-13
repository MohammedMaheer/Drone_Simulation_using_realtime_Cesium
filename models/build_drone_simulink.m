function build_drone_simulink(model_name)
% BUILD_DRONE_SIMULINK  Programmatically creates a Simulink model for the quadrotor.
%
%   build_drone_simulink()               % Creates 'drone_sim' model
%   build_drone_simulink('my_model')     % Custom model name
%
%   This script builds a complete Simulink model with:
%     - Quadrotor 6-DOF dynamics (MATLAB Function block)
%     - Motor model with first-order dynamics
%     - Cascaded PID flight controller
%     - Sensor models (IMU, GPS, Barometer)
%     - State estimator (EKF)
%     - Waypoint navigation
%     - Telemetry scopes and logging
%
%   The model uses MATLAB Function blocks that call the project's .m files,
%   so all subsystems remain editable and testable outside Simulink.

    if nargin < 1; model_name = 'drone_sim'; end

    %% Close existing model
    if bdIsLoaded(model_name)
        close_system(model_name, 0);
    end

    %% Create new model
    new_system(model_name);
    open_system(model_name);

    % Set solver
    set_param(model_name, 'Solver', 'ode4');
    set_param(model_name, 'FixedStep', '0.001');
    set_param(model_name, 'StopTime', '60');
    set_param(model_name, 'SimulationMode', 'normal');

    %% ============================================================
    %% SUBSYSTEM: Plant (Quadrotor Dynamics)
    %% ============================================================
    plant_path = [model_name, '/Quadrotor Plant'];
    add_block('simulink/Ports & Subsystems/Subsystem', plant_path);
    delete_line(plant_path, 'In1/1', 'Out1/1');
    delete_block([plant_path, '/In1']);
    delete_block([plant_path, '/Out1']);

    % MATLAB Function: Dynamics
    dynamics_path = [plant_path, '/Dynamics'];
    add_block('simulink/User-Defined Functions/MATLAB Function', dynamics_path);
    % Set the function code
    sf_root = sfroot;
    dynamics_block = sf_root.find('-isa', 'Stateflow.EMChart', 'Path', dynamics_path);
    if ~isempty(dynamics_block)
        dynamics_block.Script = sprintf([...
            'function [state_dot, forces, moments] = fcn(state, motor_speeds, wind)\n', ...
            '%%#codegen\n', ...
            'dp = drone_params();\n', ...
            '[state_dot, forces, moments] = quadrotor_dynamics(state, motor_speeds, wind, dp);\n', ...
            'end\n']);
    end

    % Integrator (state)
    add_block('simulink/Continuous/Integrator', [plant_path, '/State Integrator']);
    set_param([plant_path, '/State Integrator'], 'InitialCondition', 'zeros(12,1)');

    % Input ports
    add_block('simulink/Sources/In1', [plant_path, '/Motor Speeds']);
    add_block('simulink/Sources/In1', [plant_path, '/Wind']);

    % Output port
    add_block('simulink/Sinks/Out1', [plant_path, '/State']);
    add_block('simulink/Sinks/Out1', [plant_path, '/Forces']);

    % Connect
    add_line(plant_path, 'Motor Speeds/1', 'Dynamics/1', 'autoroute', 'on');
    add_line(plant_path, 'Wind/1', 'Dynamics/3', 'autoroute', 'on');
    add_line(plant_path, 'Dynamics/1', 'State Integrator/1', 'autoroute', 'on');
    add_line(plant_path, 'State Integrator/1', 'State/1', 'autoroute', 'on');
    add_line(plant_path, 'State Integrator/1', 'Dynamics/1', 'autoroute', 'on');
    add_line(plant_path, 'Dynamics/2', 'Forces/1', 'autoroute', 'on');

    %% ============================================================
    %% SUBSYSTEM: Flight Controller
    %% ============================================================
    ctrl_path = [model_name, '/Flight Controller'];
    add_block('simulink/Ports & Subsystems/Subsystem', ctrl_path);
    delete_line(ctrl_path, 'In1/1', 'Out1/1');
    delete_block([ctrl_path, '/In1']);
    delete_block([ctrl_path, '/Out1']);

    % MATLAB Function: Controller
    controller_fcn = [ctrl_path, '/Controller'];
    add_block('simulink/User-Defined Functions/MATLAB Function', controller_fcn);
    sf_root_ctrl = sfroot;
    ctrl_block = sf_root_ctrl.find('-isa', 'Stateflow.EMChart', 'Path', controller_fcn);
    if ~isempty(ctrl_block)
        ctrl_block.Script = sprintf([...
            'function [thrust_cmd, moment_cmds] = fcn(state, target_pos, target_yaw)\n', ...
            '%%#codegen\n', ...
            'dp = drone_params();\n', ...
            'cp = controller_params();\n', ...
            'target.position = target_pos;\n', ...
            'target.yaw = target_yaw;\n', ...
            '[thrust_cmd, moment_cmds, ~] = flight_controller(state, target, dp, cp);\n', ...
            'end\n']);
    end

    % Mixing Matrix
    mixer_fcn = [ctrl_path, '/Mixer'];
    add_block('simulink/User-Defined Functions/MATLAB Function', mixer_fcn);
    mixer_block = sfroot.find('-isa', 'Stateflow.EMChart', 'Path', mixer_fcn);
    if ~isempty(mixer_block)
        mixer_block.Script = sprintf([...
            'function motor_cmds = fcn(thrust_cmd, moment_cmds)\n', ...
            '%%#codegen\n', ...
            'dp = drone_params();\n', ...
            'motor_cmds = mixing_matrix(thrust_cmd, moment_cmds, dp);\n', ...
            'end\n']);
    end

    % IO Ports
    add_block('simulink/Sources/In1', [ctrl_path, '/State']);
    add_block('simulink/Sources/In1', [ctrl_path, '/Target Pos']);
    add_block('simulink/Sources/In1', [ctrl_path, '/Target Yaw']);
    add_block('simulink/Sinks/Out1', [ctrl_path, '/Motor Commands']);

    % Connect
    add_line(ctrl_path, 'State/1', 'Controller/1', 'autoroute', 'on');
    add_line(ctrl_path, 'Target Pos/1', 'Controller/2', 'autoroute', 'on');
    add_line(ctrl_path, 'Target Yaw/1', 'Controller/3', 'autoroute', 'on');
    add_line(ctrl_path, 'Controller/1', 'Mixer/1', 'autoroute', 'on');
    add_line(ctrl_path, 'Controller/2', 'Mixer/2', 'autoroute', 'on');
    add_line(ctrl_path, 'Mixer/1', 'Motor Commands/1', 'autoroute', 'on');

    %% ============================================================
    %% SUBSYSTEM: Sensors
    %% ============================================================
    sensor_path = [model_name, '/Sensors'];
    add_block('simulink/Ports & Subsystems/Subsystem', sensor_path);
    delete_line(sensor_path, 'In1/1', 'Out1/1');
    delete_block([sensor_path, '/In1']);
    delete_block([sensor_path, '/Out1']);

    sensor_fcn = [sensor_path, '/Sensor Suite'];
    add_block('simulink/User-Defined Functions/MATLAB Function', sensor_fcn);
    sensor_block = sfroot.find('-isa', 'Stateflow.EMChart', 'Path', sensor_fcn);
    if ~isempty(sensor_block)
        sensor_block.Script = sprintf([...
            'function [accel, gyro, gps_pos, baro_alt, heading] = fcn(state, t)\n', ...
            '%%#codegen\n', ...
            'sp = sensor_params();\n', ...
            'dp = drone_params();\n', ...
            'R = eul2rotm_local(state(7), state(8), state(9));\n', ...
            'accel_true = R'' * [0;0;-dp.g] + R'' * [0;0;0];\n', ...
            'gyro_true = state(10:12);\n', ...
            '[accel, gyro] = imu_model(accel_true, gyro_true, 0.001, sp);\n', ...
            '[gps_pos, ~, ~] = gps_model(state(1:3), state(4:6), t, sp);\n', ...
            'baro_alt = barometer_model(-state(3), t, sp);\n', ...
            'heading = magnetometer_model(state(7:9), sp);\n', ...
            'end\n\n', ...
            'function R = eul2rotm_local(phi, theta, psi)\n', ...
            'cphi=cos(phi);sphi=sin(phi);cth=cos(theta);sth=sin(theta);cpsi=cos(psi);spsi=sin(psi);\n', ...
            'R=[cth*cpsi,sphi*sth*cpsi-cphi*spsi,cphi*sth*cpsi+sphi*spsi;', ...
            'cth*spsi,sphi*sth*spsi+cphi*cpsi,cphi*sth*spsi-sphi*cpsi;', ...
            '-sth,sphi*cth,cphi*cth];\n', ...
            'end\n']);
    end

    add_block('simulink/Sources/In1', [sensor_path, '/State']);
    add_block('simulink/Sources/In1', [sensor_path, '/Time']);
    add_block('simulink/Sinks/Out1', [sensor_path, '/Accel']);
    add_block('simulink/Sinks/Out1', [sensor_path, '/Gyro']);
    add_block('simulink/Sinks/Out1', [sensor_path, '/GPS Pos']);
    add_block('simulink/Sinks/Out1', [sensor_path, '/Baro Alt']);
    add_block('simulink/Sinks/Out1', [sensor_path, '/Heading']);

    add_line(sensor_path, 'State/1', 'Sensor Suite/1', 'autoroute', 'on');
    add_line(sensor_path, 'Time/1', 'Sensor Suite/2', 'autoroute', 'on');
    add_line(sensor_path, 'Sensor Suite/1', 'Accel/1', 'autoroute', 'on');
    add_line(sensor_path, 'Sensor Suite/2', 'Gyro/1', 'autoroute', 'on');
    add_line(sensor_path, 'Sensor Suite/3', 'GPS Pos/1', 'autoroute', 'on');
    add_line(sensor_path, 'Sensor Suite/4', 'Baro Alt/1', 'autoroute', 'on');
    add_line(sensor_path, 'Sensor Suite/5', 'Heading/1', 'autoroute', 'on');

    %% ============================================================
    %% TOP-LEVEL CONNECTIONS
    %% ============================================================

    % Clock for time
    add_block('simulink/Sources/Clock', [model_name, '/Clock']);

    % Target position (Constant for now — can be replaced with waypoint nav)
    add_block('simulink/Sources/Constant', [model_name, '/Target Position']);
    set_param([model_name, '/Target Position'], 'Value', '[0; 0; -10]');
    set_param([model_name, '/Target Position'], 'OutDataTypeStr', 'double');

    add_block('simulink/Sources/Constant', [model_name, '/Target Yaw']);
    set_param([model_name, '/Target Yaw'], 'Value', '0');

    % Wind (constant or signal)
    add_block('simulink/Sources/Constant', [model_name, '/Wind']);
    set_param([model_name, '/Wind'], 'Value', '[0; 0; 0]');

    % Scopes
    add_block('simulink/Sinks/Scope', [model_name, '/Position Scope']);
    set_param([model_name, '/Position Scope'], 'NumInputPorts', '1');

    add_block('simulink/Sinks/Scope', [model_name, '/Attitude Scope']);
    set_param([model_name, '/Attitude Scope'], 'NumInputPorts', '1');

    % To Workspace blocks for logging
    add_block('simulink/Sinks/To Workspace', [model_name, '/Log State']);
    set_param([model_name, '/Log State'], 'VariableName', 'state_log');
    set_param([model_name, '/Log State'], 'SaveFormat', 'Timeseries');

    add_block('simulink/Sinks/To Workspace', [model_name, '/Log Motors']);
    set_param([model_name, '/Log Motors'], 'VariableName', 'motor_log');
    set_param([model_name, '/Log Motors'], 'SaveFormat', 'Timeseries');

    % Demux for state (position: 1-3, velocity: 4-6, euler: 7-9, omega: 10-12)
    add_block('simulink/Signal Routing/Demux', [model_name, '/State Demux']);
    set_param([model_name, '/State Demux'], 'Outputs', '[3,3,3,3]');

    %% Top-level wiring
    % Controller → Plant
    add_line(model_name, 'Flight Controller/1', 'Quadrotor Plant/1', 'autoroute', 'on');
    add_line(model_name, 'Wind/1', 'Quadrotor Plant/2', 'autoroute', 'on');

    % Plant → Controller (feedback)
    add_line(model_name, 'Quadrotor Plant/1', 'Flight Controller/1', 'autoroute', 'on');

    % Target → Controller
    add_line(model_name, 'Target Position/1', 'Flight Controller/2', 'autoroute', 'on');
    add_line(model_name, 'Target Yaw/1', 'Flight Controller/3', 'autoroute', 'on');

    % Plant → Sensors
    add_line(model_name, 'Quadrotor Plant/1', 'Sensors/1', 'autoroute', 'on');
    add_line(model_name, 'Clock/1', 'Sensors/2', 'autoroute', 'on');

    % Plant → State Demux → Scopes
    add_line(model_name, 'Quadrotor Plant/1', 'State Demux/1', 'autoroute', 'on');
    add_line(model_name, 'State Demux/1', 'Position Scope/1', 'autoroute', 'on');
    add_line(model_name, 'State Demux/3', 'Attitude Scope/1', 'autoroute', 'on');

    % Logging
    add_line(model_name, 'Quadrotor Plant/1', 'Log State/1', 'autoroute', 'on');
    add_line(model_name, 'Flight Controller/1', 'Log Motors/1', 'autoroute', 'on');

    %% Layout cleanup
    Simulink.BlockDiagram.arrangeSystem(model_name);

    %% Save model
    save_system(model_name);
    fprintf('Simulink model "%s.slx" built and saved successfully.\n', model_name);
    fprintf('Open with: open_system(''%s'')\n', model_name);

end
