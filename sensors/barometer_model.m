function alt_baro = barometer_model(alt_true, t, sp)
% BAROMETER_MODEL  Simulates barometric altimeter with noise and drift.
%
%   alt_baro = barometer_model(alt_true, t, sp)
%
%   Inputs:
%     alt_true - True altitude [m] (positive up)
%     t        - Current sim time [s]
%     sp       - sensor_params struct
%
%   Outputs:
%     alt_baro - Measured barometric altitude [m]

    persistent last_update_time last_alt drift
    if isempty(last_update_time)
        last_update_time = -1;
        last_alt = alt_true;
        drift = 0;
    end

    update_period = 1 / sp.baro.update_rate;
    if (t - last_update_time) >= update_period
        last_update_time = t;

        % Accumulate slow drift
        drift = drift + sp.baro.drift_rate * update_period;

        % White noise
        noise = sp.baro.noise_std * randn;

        last_alt = alt_true + drift + noise;
    end

    alt_baro = last_alt;

end
