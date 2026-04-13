# Multirotor Drone Simulation — MATLAB

A comprehensive, competition-grade multirotor UAV simulation covering high-fidelity flight
dynamics, cascaded PID control, navigation, realistic sensor modeling, state estimation,
and interactive 3D visualization. Supports 3/4/6/8-motor configurations with 5 presets.

## Features

- **N-Motor Support**: Tricopter, quadcopter (X/+), hexacopter (flat/Y), octocopter (flat/X)
- **High-Fidelity Physics**: 6-DOF rigid body, gyroscopic precession, blade flapping, hub drag
- **Dryden Wind Turbulence**: MIL-DTL-9490E compliant colored noise turbulence model
- **Realistic Motor Model**: Voltage sag, efficiency bell-curve, current limiting, back-EMF
- **Battery Thermal Model**: I²R heating, convective cooling, thermal cutoff protection
- **Propeller Vibration**: 1/rev + 2/rev imbalance with structural resonance
- **Sensor Suite**: IMU, GPS (with latency/dropout), barometer, magnetometer
- **EKF State Estimator**: 12-state Extended Kalman Filter with sensor fusion
- **Interactive 3D Flight**: Real-time keyboard-controlled flight with HUD & strip charts
- **Cheeseman-Bennett Ground Effect**: Peer-reviewed rotor-altitude model
- **5 Preset Drones**: mini_quad (250mm), standard_quad (450mm), heavy_hex (680mm), octo_lift (1000mm), micro_tri (180mm)
- **Comprehensive Test Suite**: Unit + integration tests for all core modules

## Project Structure

```
SimuLink_College/
├── init_project.m              % Project initialization — run first
├── fly_drone.m                 % Interactive launcher menu
├── run_simulation.m            % Classic simulation entry point
├── PHYSICS_GUIDE.md            % Complete mathematical reference
│
├── params/                     % Configuration
│   ├── drone_config.m          % Central N-motor config builder (5 presets)
│   ├── drone_params.m          % Legacy 4-motor parameters
│   ├── controller_params.m     % PID gains for cascaded controller
│   ├── sensor_params.m         % Sensor noise & timing parameters
│   └── sim_params.m            % Simulation settings
│
├── models/                     % Physics models
│   ├── multirotor_dynamics.m   % Generalized N-motor 6-DOF dynamics
│   ├── quadrotor_dynamics.m    % Legacy 4-motor dynamics
│   ├── motor_model_precise.m   % High-fidelity motor with battery model
│   ├── motor_model.m           % Simple motor model
│   ├── battery_thermal_model.m % LiPo thermal dynamics
│   ├── propeller_vibration_model.m % Vibration from prop imbalance
│   ├── mixing_matrix_n.m       % N-motor control allocation
│   └── mixing_matrix.m         % Legacy 4-motor mixing
│
├── environment/                % Environmental models
│   └── dryden_wind_model.m     % MIL-DTL-9490E Dryden turbulence
│
├── control/                    % Flight controller
│   ├── flight_controller.m     % Cascaded PID wrapper
│   ├── attitude_controller.m   % Roll/Pitch/Yaw attitude PID
│   ├── position_controller.m   % X/Y position PID
│   └── altitude_controller.m   % Altitude hold PID
│
├── navigation/                 % Navigation & guidance
│   ├── waypoint_manager.m      % Waypoint sequencing
│   ├── path_planner.m          % Path interpolation
│   └── mission_profiles.m      % Pre-defined missions
│
├── sensors/                    % Sensor simulation
│   ├── imu_model.m             % Accelerometer + Gyroscope
│   ├── gps_model.m             % GPS position + velocity
│   ├── barometer_model.m       % Barometric altimeter
│   ├── magnetometer_model.m    % Magnetometer (heading)
│   ├── state_estimator.m       % 12-state EKF fusion
│   └── sensor_latency_model.m  % Transport delay + dropout
│
├── telemetry/                  % Data logging & monitoring
│   ├── telemetry_logger.m      % In-sim data recording
│   ├── telemetry_dashboard.m   % Live telemetry display
│   └── post_flight_analysis.m  % Post-sim analysis
│
├── visualization/              % 3D visualization
│   ├── live_drone_sim.m        % Interactive 3D flight simulator
│   ├── flight_analysis.m       % Post-flight 6-panel dashboard
│   ├── drone_3d_plot.m         % Static 3D drone rendering
│   ├── animate_flight.m        % Animated flight replay
│   └── plot_flight_data.m      % Standard telemetry plots
│
├── tests/                      % Test suite
│   ├── run_all_tests.m         % Test runner (all suites)
│   ├── unit/
│   │   ├── test_config.m       % Config validation tests
│   │   ├── test_motor_model.m  % Motor physics tests
│   │   ├── test_dynamics.m     % 6-DOF dynamics tests
│   │   └── test_wind_model.m   % Dryden turbulence tests
│   ├── integration/
│   │   └── test_hover_stability.m % Multi-preset hover tests
│   ├── test_hover.m            % Legacy hover test
│   ├── test_waypoint_nav.m     % Waypoint following test
│   ├── test_wind_disturbance.m % Wind rejection test
│   └── test_sensor_failure.m   % Sensor degradation test
│
└── utilities/                  % Math utilities
    ├── rotation_utils.m        % Rotation matrices
    ├── quaternion_utils.m      % Quaternion operations
    └── coord_transforms.m      % NED ↔ Body transforms
```

## Quick Start

1. Open MATLAB and navigate to the `SimuLink_College` folder.
2. Run `init_project` to add all folders to the MATLAB path.
3. Run `fly_drone` to launch the interactive flight menu.
4. Or `fly_drone('quick')` for instant flight with default quad.
5. Run `run_all_tests` to execute the full test suite.

## Interactive Flight Controls

| Key | Action |
|-----|--------|
| W/S | Pitch forward/backward |
| A/D | Roll left/right |
| Q/E | Yaw left/right |
| Space/Shift | Climb/descend |
| M | Toggle auto/manual mode |
| H | Toggle auto-hover |
| R | Reset position |
| 1 | Toggle wind (Dryden turbulence) |
| 2 | Toggle trail |
| 3 | Toggle camera follow |
| 4 | Toggle precise battery |
| TAB | Cycle HUD pages |
| ESC | Quit |

## Physics Models

### Flight Dynamics
- Full 6-DOF rigid-body dynamics for N-motor multirotors
- RK4 integration at 500 Hz for stability
- Inertia tensor computed from component geometry (cuboid body + rod arms + point masses)
- Gyroscopic precession from spinning propellers
- Blade flapping with advance ratio correction (Leishman/Prouty)
- Hub drag (parasitic drag in forward flight)
- Gimbal lock protection at extreme pitch angles

### Motor & Battery
- Voltage-dependent speed ceiling with back-EMF limiting
- Gaussian bell-curve efficiency (peaks at 65% throttle, calibrated to T-Motor data)
- Per-motor and total pack current limiting (C-rating)
- LiPo discharge curve (empirical 3-region polynomial)
- Battery thermal model: I²R heating, convective cooling, thermal cutoff
- Motor winding temperature tracking with efficiency derating

### Aerodynamics
- Cheeseman-Bennett ground effect (1955, peer-reviewed)
- Quadratic aerodynamic drag (body + hub)
- Dryden continuous wind turbulence (MIL-DTL-9490E)
- Propeller vibration (1/rev static + 2/rev dynamic imbalance)
- Structural resonance amplification near frame natural frequency

### Control System
- Cascaded PID: Position (50 Hz) → Attitude (250 Hz) → Rate (1000 Hz)
- Full PID (P+I+D) on all loops including rate controller
- Integral anti-windup on all integrators
- Auto-tuned gains from drone configuration
- Gravity feedforward on altitude controller

### Sensors & Estimation
- IMU with bias random walk, white noise, and saturation
- GPS with realistic noise, HDOP, latency (100ms), and dropout events
- Barometer with drift and temperature sensitivity
- Magnetometer with hard/soft iron distortion
- 12-state Extended Kalman Filter with sensor fusion
- Sensor latency model (configurable per-sensor)

### Navigation
- Waypoint sequencing with accept radius
- Loiter mode with configurable dwell time
- Multiple mission profiles (square, circle, figure-8)

## Test Suite

Run `run_all_tests` to execute:
- **Unit tests**: Config validation (5 presets), motor model bounds, dynamics physics, wind model statistics
- **Integration tests**: Hover stability for all presets, hover-in-wind robustness

## Documentation

See [PHYSICS_GUIDE.md](PHYSICS_GUIDE.md) for complete mathematical derivations.

## Requirements
- MATLAB R2020a or later
- No additional toolboxes required for core simulation
- Simulink (optional, for block diagram model)
