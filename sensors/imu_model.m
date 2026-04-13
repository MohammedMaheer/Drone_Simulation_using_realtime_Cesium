function [accel_meas, gyro_meas] = imu_model(accel_true, gyro_true, dt, sp)
% IMU_MODEL  Simulates accelerometer and gyroscope with realistic noise.
%
%   [accel_meas, gyro_meas] = imu_model(accel_true, gyro_true, dt, sp)
%
%   Inputs:
%     accel_true - [3x1] true specific force in body frame [m/s^2]
%     gyro_true  - [3x1] true angular velocity in body frame [rad/s]
%     dt         - time step [s]
%     sp         - sensor_params struct
%
%   Outputs:
%     accel_meas - [3x1] measured acceleration [m/s^2]
%     gyro_meas  - [3x1] measured angular velocity [rad/s]

    persistent accel_bias gyro_bias
    if isempty(accel_bias)
        accel_bias = sp.accel.bias_init;
        gyro_bias  = sp.gyro.bias_init;
    end

    %% Bias random walk (slow drift)
    accel_bias = accel_bias + sp.accel.bias_stability * sqrt(dt) * randn(3,1);
    gyro_bias  = gyro_bias  + sp.gyro.bias_stability  * sqrt(dt) * randn(3,1);

    %% White noise
    accel_noise = sp.accel.noise_density * sqrt(1/dt) * randn(3,1);
    gyro_noise  = sp.gyro.noise_density  * sqrt(1/dt) * randn(3,1);

    %% Measurement = truth + bias + noise
    accel_meas = accel_true + accel_bias + accel_noise;
    gyro_meas  = gyro_true  + gyro_bias  + gyro_noise;

    %% Saturation
    accel_meas = max(-sp.accel.range, min(sp.accel.range, accel_meas));
    gyro_meas  = max(-sp.gyro.range,  min(sp.gyro.range,  gyro_meas));

end
