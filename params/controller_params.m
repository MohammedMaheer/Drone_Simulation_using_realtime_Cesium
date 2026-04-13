function params = controller_params()
% CONTROLLER_PARAMS  PID gains and limits for the cascaded flight controller.
%
%   params = controller_params() returns a struct containing PID gains for
%   position, altitude, attitude, and angular rate control loops.

    %% ===== Position Controller (Outer Loop) =====
    % Outputs desired roll/pitch angles from position error
    params.pos_x.Kp = 1.2;
    params.pos_x.Ki = 0.05;
    params.pos_x.Kd = 0.8;
    params.pos_x.max_angle = deg2rad(25);  % Max tilt command [rad]

    params.pos_y.Kp = 1.2;
    params.pos_y.Ki = 0.05;
    params.pos_y.Kd = 0.8;
    params.pos_y.max_angle = deg2rad(25);

    %% ===== Altitude Controller =====
    % Outputs desired total thrust from altitude error
    params.alt.Kp = 4.0;
    params.alt.Ki = 0.8;
    params.alt.Kd = 3.0;
    params.alt.max_thrust   = 30;    % Max thrust command [N]
    params.alt.min_thrust   = 2;     % Min thrust command [N]
    params.alt.max_climb    = 3.0;   % Max climb rate [m/s]
    params.alt.max_descent  = 2.0;   % Max descent rate [m/s]

    %% ===== Attitude Controller (Middle Loop) =====
    % Outputs desired angular rates from attitude error
    params.roll.Kp  = 6.5;
    params.roll.Ki  = 0.5;
    params.roll.Kd  = 1.2;

    params.pitch.Kp = 6.5;
    params.pitch.Ki = 0.5;
    params.pitch.Kd = 1.2;

    params.yaw.Kp   = 4.0;
    params.yaw.Ki   = 0.3;
    params.yaw.Kd   = 0.8;

    %% ===== Rate Controller (Inner Loop) =====
    % Outputs torque commands from angular rate error
    params.roll_rate.Kp  = 0.15;
    params.roll_rate.Ki  = 0.01;
    params.roll_rate.Kd  = 0.002;

    params.pitch_rate.Kp = 0.15;
    params.pitch_rate.Ki = 0.01;
    params.pitch_rate.Kd = 0.002;

    params.yaw_rate.Kp   = 0.30;
    params.yaw_rate.Ki   = 0.02;
    params.yaw_rate.Kd   = 0.001;

    %% ===== Rate Limits =====
    params.max_roll_rate  = deg2rad(250);   % [rad/s]
    params.max_pitch_rate = deg2rad(250);
    params.max_yaw_rate   = deg2rad(180);

    %% ===== Integrator Anti-Windup =====
    params.integrator_max = 5.0;   % Generic integrator saturation limit

    %% ===== Control Loop Rates =====
    params.position_rate = 50;     % Position loop [Hz]
    params.attitude_rate = 250;    % Attitude loop [Hz]
    params.rate_rate     = 1000;   % Rate loop [Hz]

end
