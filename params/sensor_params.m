function params = sensor_params()
% SENSOR_PARAMS  Noise, bias, and timing parameters for simulated sensors.
%
%   params = sensor_params() returns a struct with realistic noise models
%   for IMU, GPS, barometer, and magnetometer.

    %% ===== IMU — Accelerometer =====
    params.accel.noise_density  = 0.004;    % Noise density [m/s^2/sqrt(Hz)]
    params.accel.bias_stability = 0.05;     % Bias instability [m/s^2]
    params.accel.bias_init      = [0.02; -0.01; 0.03];  % Initial bias [m/s^2]
    params.accel.range          = 16 * 9.81; % Full-scale range [m/s^2] (±16g)
    params.accel.update_rate    = 1000;      % Update rate [Hz]

    %% ===== IMU — Gyroscope =====
    params.gyro.noise_density   = 0.0035;   % Noise density [rad/s/sqrt(Hz)]
    params.gyro.bias_stability  = 0.001;    % Bias instability [rad/s]
    params.gyro.bias_init       = [0.005; -0.003; 0.002]; % Initial bias [rad/s]
    params.gyro.range           = deg2rad(2000);  % Full-scale range [rad/s]
    params.gyro.update_rate     = 1000;      % Update rate [Hz]

    %% ===== GPS =====
    params.gps.pos_noise_std    = 1.5;      % Position noise std dev [m]
    params.gps.vel_noise_std    = 0.3;      % Velocity noise std dev [m/s]
    params.gps.update_rate      = 5;        % Update rate [Hz]
    params.gps.latency          = 0.1;      % Measurement latency [s]
    params.gps.hdop             = 1.2;      % Horizontal dilution of precision
    params.gps.satellite_min    = 6;        % Minimum satellites for fix

    %% ===== Barometer =====
    params.baro.noise_std       = 0.5;      % Altitude noise std dev [m]
    params.baro.drift_rate      = 0.01;     % Drift rate [m/s]
    params.baro.temp_sensitivity = 0.02;    % Temperature sensitivity [m/°C]
    params.baro.update_rate     = 50;       % Update rate [Hz]

    %% ===== Magnetometer =====
    params.mag.noise_std        = 0.005;    % Noise std dev [Gauss]
    params.mag.hard_iron        = [0.02; -0.01; 0.015]; % Hard iron offset [Gauss]
    params.mag.soft_iron        = eye(3) + 0.01 * randn(3); % Soft iron matrix
    params.mag.declination      = deg2rad(10);  % Magnetic declination [rad]
    params.mag.update_rate      = 100;      % Update rate [Hz]

    %% ===== State Estimator Tuning =====
    params.estimator.type       = 'EKF';    % 'complementary' or 'EKF'
    params.estimator.alpha      = 0.98;     % Complementary filter weight
    % EKF process noise
    params.estimator.Q_pos      = 0.01;     % Position process noise
    params.estimator.Q_vel      = 0.1;      % Velocity process noise
    params.estimator.Q_att      = 0.001;    % Attitude process noise
    params.estimator.Q_bias     = 0.0001;   % Bias process noise

end
