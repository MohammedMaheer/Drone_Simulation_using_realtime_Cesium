function [est_state, P] = state_estimator(sensor_data, dt, sp, dp)
% STATE_ESTIMATOR  Extended Kalman Filter for drone state estimation.
%
%   [est_state, P] = state_estimator(sensor_data, dt, sp, dp)
%
%   Implements a 12-state EKF fusing IMU, GPS, barometer, and magnetometer.
%
%   State vector: [x y z vx vy vz phi theta psi bias_ax bias_ay bias_az]'
%
%   Inputs:
%     sensor_data - struct with fields:
%                     .accel     - [3x1] accelerometer [m/s^2]
%                     .gyro      - [3x1] gyroscope [rad/s]
%                     .gps_pos   - [3x1] GPS position NED [m]
%                     .gps_vel   - [3x1] GPS velocity NED [m/s]
%                     .gps_valid - boolean
%                     .baro_alt  - barometric altitude [m]
%                     .mag_heading - magnetometer heading [rad]
%     dt          - time step [s]
%     sp          - sensor_params struct
%     dp          - drone_params struct
%
%   Outputs:
%     est_state - [12x1] estimated state
%     P         - [12x12] state covariance matrix

    persistent x_est P_est initialized
    if isempty(initialized)
        x_est = zeros(12, 1);
        P_est = eye(12) * 10;
        initialized = true;
    end

    n = 12;  % State dimension

    %% ===== PREDICTION (IMU-driven) =====
    % Unpack current estimate
    pos_est   = x_est(1:3);
    vel_est   = x_est(4:6);
    euler_est = x_est(7:9);
    bias_est  = x_est(10:12);

    phi = euler_est(1); theta = euler_est(2); psi = euler_est(3);

    % Correct accelerometer for estimated bias
    accel_corrected = sensor_data.accel - bias_est;

    % Rotation matrix (Body → NED)
    R = euler_to_dcm(phi, theta, psi);

    % Transform acceleration to NED and remove gravity
    accel_ned = R * accel_corrected + [0; 0; dp.g];

    % Integrate angular rates (Euler approximation)
    gyro = sensor_data.gyro;
    E = euler_rate_matrix(phi, theta);
    euler_dot = E * gyro;

    % Predict state
    x_pred = x_est;
    x_pred(1:3)  = pos_est + vel_est * dt;
    x_pred(4:6)  = vel_est + accel_ned * dt;
    x_pred(7:9)  = euler_est + euler_dot * dt;
    x_pred(10:12) = bias_est;  % Bias modeled as random walk

    % Wrap angles
    x_pred(7:9) = wrap_angles(x_pred(7:9));

    % Process noise
    Q = zeros(n);
    Q(1:3,1:3)   = eye(3) * sp.estimator.Q_pos;
    Q(4:6,4:6)   = eye(3) * sp.estimator.Q_vel;
    Q(7:9,7:9)   = eye(3) * sp.estimator.Q_att;
    Q(10:12,10:12) = eye(3) * sp.estimator.Q_bias;

    % Linearized state transition (simplified)
    F = eye(n);
    F(1:3, 4:6) = eye(3) * dt;
    F(4:6, 7:9) = skew_symmetric(accel_ned) * dt;  % Approximate

    P_pred = F * P_est * F' + Q;

    %% ===== UPDATE (GPS + Baro + Mag) =====
    x_upd = x_pred;
    P_upd = P_pred;

    % GPS Position update
    if sensor_data.gps_valid
        H_gps_pos = zeros(3, n);
        H_gps_pos(1:3, 1:3) = eye(3);
        R_gps_pos = eye(3) * (sp.gps.pos_noise_std * sp.gps.hdop)^2;
        z_gps = sensor_data.gps_pos;
        y_gps = z_gps - x_upd(1:3);
        [x_upd, P_upd] = ekf_update(x_upd, P_upd, y_gps, H_gps_pos, R_gps_pos);

        % GPS Velocity update
        H_gps_vel = zeros(3, n);
        H_gps_vel(1:3, 4:6) = eye(3);
        R_gps_vel = eye(3) * sp.gps.vel_noise_std^2;
        z_vel = sensor_data.gps_vel;
        y_vel = z_vel - x_upd(4:6);
        [x_upd, P_upd] = ekf_update(x_upd, P_upd, y_vel, H_gps_vel, R_gps_vel);
    end

    % Barometer altitude update
    H_baro = zeros(1, n);
    H_baro(1, 3) = -1;  % Baro measures altitude = -z_ned
    R_baro = sp.baro.noise_std^2;
    z_baro = sensor_data.baro_alt;
    y_baro = z_baro - (-x_upd(3));
    [x_upd, P_upd] = ekf_update(x_upd, P_upd, y_baro, H_baro, R_baro);

    % Magnetometer heading update
    H_mag = zeros(1, n);
    H_mag(1, 9) = 1;  % Measures yaw
    R_mag = (sp.mag.noise_std * 50)^2;
    z_mag = sensor_data.mag_heading;
    y_mag = atan2(sin(z_mag - x_upd(9)), cos(z_mag - x_upd(9)));  % Angle wrapping
    [x_upd, P_upd] = ekf_update(x_upd, P_upd, y_mag, H_mag, R_mag);

    %% Store and return
    x_est = x_upd;
    P_est = P_upd;
    est_state = x_est;
    P = P_est;

end


function [x, P] = ekf_update(x, P, y, H, R)
% Standard EKF measurement update
    S = H * P * H' + R;
    K = P * H' / S;
    x = x + K * y;
    P = (eye(length(x)) - K * H) * P;
    P = (P + P') / 2;  % Enforce symmetry
end


function R = euler_to_dcm(phi, theta, psi)
    cphi = cos(phi); sphi = sin(phi);
    cth = cos(theta); sth = sin(theta);
    cpsi = cos(psi); spsi = sin(psi);

    R = [cth*cpsi,  sphi*sth*cpsi-cphi*spsi,  cphi*sth*cpsi+sphi*spsi;
         cth*spsi,  sphi*sth*spsi+cphi*cpsi,  cphi*sth*spsi-sphi*cpsi;
         -sth,      sphi*cth,                  cphi*cth                ];
end


function E = euler_rate_matrix(phi, theta)
    cphi = cos(phi); sphi = sin(phi);
    cth = cos(theta); tth = tan(theta);

    E = [1, sphi*tth, cphi*tth;
         0, cphi,     -sphi;
         0, sphi/cth, cphi/cth];
end


function S = skew_symmetric(v)
    S = [ 0,    -v(3),  v(2);
          v(3),  0,    -v(1);
         -v(2),  v(1),  0   ];
end


function a = wrap_angles(a)
    a = atan2(sin(a), cos(a));
end
