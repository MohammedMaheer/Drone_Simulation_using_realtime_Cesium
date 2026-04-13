function heading_mag = magnetometer_model(euler_true, sp)
% MAGNETOMETER_MODEL  Simulates magnetometer heading measurement.
%
%   heading_mag = magnetometer_model(euler_true, sp)
%
%   Inputs:
%     euler_true - [3x1] true Euler angles [phi; theta; psi] [rad]
%     sp         - sensor_params struct
%
%   Outputs:
%     heading_mag - Measured magnetic heading [rad]

    %% True heading
    true_heading = euler_true(3);

    %% Add magnetic declination
    heading_with_decl = true_heading + sp.mag.declination;

    %% Hard iron distortion (simplified as heading offset)
    hard_iron_offset = atan2(sp.mag.hard_iron(2), sp.mag.hard_iron(1));

    %% Noise
    noise = sp.mag.noise_std * randn * 50;  % Scale noise to heading domain

    %% Measured heading
    heading_mag = heading_with_decl + hard_iron_offset + noise;

    % Wrap to [-pi, pi]
    heading_mag = atan2(sin(heading_mag), cos(heading_mag));

end
