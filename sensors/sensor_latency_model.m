function [delayed_meas, dropout] = sensor_latency_model(measurement, sensor_type, dt, sp)
% SENSOR_LATENCY_MODEL  Adds realistic latency and dropout to sensor measurements.
%
%   [delayed_meas, dropout] = sensor_latency_model(measurement, sensor_type, dt, sp)
%
%   Models two critical real-world sensor phenomena:
%     1) Transport delay: GPS has ~100ms lag, baro ~20ms, mag ~10ms
%     2) Measurement dropout: GPS can lose fix, baro can spike
%
%   Inputs:
%     measurement - [Nx1] current true measurement vector
%     sensor_type - 'gps', 'baro', 'mag', or 'imu'
%     dt          - simulation time step [s]
%     sp          - sensor_params struct
%
%   Outputs:
%     delayed_meas - [Nx1] delayed measurement (from buffer)
%     dropout      - boolean, true if measurement is invalid this step
%
%   Usage:
%     [gps_delayed, gps_lost] = sensor_latency_model(gps_raw, 'gps', dt, sp);
%     if ~gps_lost
%         % Use gps_delayed in estimator
%     end

    persistent buffers buf_idx buf_sizes dropout_counters
    if isempty(buffers)
        buffers = struct();
        buf_idx = struct();
        buf_sizes = struct();
        dropout_counters = struct();
    end

    %% Get latency and dropout parameters for this sensor type
    switch lower(sensor_type)
        case 'gps'
            if isfield(sp, 'gps') && isfield(sp.gps, 'latency')
                latency = sp.gps.latency;  % ~100ms
            else
                latency = 0.1;
            end
            dropout_prob = 0.001;  % 0.1% per step (~2s mean time between events at 500Hz)
            dropout_duration = 0.5;  % 0.5s GPS outage

        case 'baro'
            latency = 0.02;   % 20ms
            dropout_prob = 0.0005;
            dropout_duration = 0.1;

        case 'mag'
            latency = 0.01;   % 10ms
            dropout_prob = 0.0002;
            dropout_duration = 0.05;

        case 'imu'
            latency = 0.001;  % 1ms (negligible)
            dropout_prob = 0.0001;
            dropout_duration = 0.002;

        otherwise
            latency = 0;
            dropout_prob = 0;
            dropout_duration = 0;
    end

    %% Initialize ring buffer for this sensor if needed
    field = lower(sensor_type);
    if ~isfield(buffers, field)
        n_samples = max(1, ceil(latency / dt));
        buffers.(field) = repmat(measurement(:)', n_samples, 1);
        buf_idx.(field) = 1;
        buf_sizes.(field) = n_samples;
        dropout_counters.(field) = 0;
    end

    %% Ring buffer read (oldest sample = delayed measurement)
    idx = buf_idx.(field);
    delayed_meas = buffers.(field)(idx, :)';

    %% Ring buffer write (current sample)
    buffers.(field)(idx, :) = measurement(:)';
    buf_idx.(field) = mod(idx, buf_sizes.(field)) + 1;

    %% Dropout simulation
    if dropout_counters.(field) > 0
        % Currently in a dropout event
        dropout_counters.(field) = dropout_counters.(field) - dt;
        dropout = true;
    elseif rand < dropout_prob
        % New dropout event
        dropout_counters.(field) = dropout_duration;
        dropout = true;
    else
        dropout = false;
    end

end
