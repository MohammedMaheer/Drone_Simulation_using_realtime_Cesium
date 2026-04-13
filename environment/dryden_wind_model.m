function [wind_body, turb_body] = dryden_wind_model(t, dt, altitude, airspeed, ...
        wind_steady, turb_intensity)
% DRYDEN_WIND_MODEL  MIL-DTL-9490E / MIL-HDBK-1797 Dryden turbulence model.
%
%   [wind_body, turb_body] = dryden_wind_model(t, dt, altitude, airspeed, ...
%       wind_steady, turb_intensity)
%
%   Generates realistic wind disturbances using the Dryden continuous
%   turbulence model, which produces colored noise with the correct
%   power spectral density for atmospheric turbulence.
%
%   Inputs:
%     t               - simulation time [s]
%     dt              - time step [s]
%     altitude        - AGL altitude [m] (> 0)
%     airspeed        - total airspeed [m/s]
%     wind_steady     - [3x1] steady-state wind in NED [m/s]
%     turb_intensity  - 'light', 'moderate', 'severe', or sigma [m/s]
%
%   Outputs:
%     wind_body - [3x1] total wind (steady + turbulence) in NED [m/s]
%     turb_body - [3x1] turbulence component only in NED [m/s]
%
%   Reference:
%     MIL-DTL-9490E, Section 3.7.2 (Dryden Turbulence Model)
%     MIL-HDBK-1797, Appendix A, Section A.8.12

    persistent state_u state_v state_w initialized prev_t
    if isempty(initialized) || (t < prev_t)  % Reset on time reversal
        state_u = [0; 0];
        state_v = [0; 0; 0];
        state_w = [0; 0; 0];
        initialized = true;
    end
    prev_t = t;

    %% Turbulence intensity (sigma)
    if ischar(turb_intensity) || isstring(turb_intensity)
        switch lower(char(turb_intensity))
            case 'light',    sigma_w = 0.5;
            case 'moderate', sigma_w = 2.0;
            case 'severe',   sigma_w = 5.0;
            otherwise,       sigma_w = 1.5;
        end
    else
        sigma_w = turb_intensity;
    end

    %% Dryden scale lengths (low altitude, < 300m AGL)
    h = max(3, min(altitude, 300));  % Clamp altitude for scale length calculation

    % MIL-HDBK-1797 low-altitude model
    Lu = h / (0.177 + 0.000823 * h)^1.2;
    Lv = Lu;
    Lw = h;

    % Turbulence intensities (u,v scaled from w)
    sigma_u = sigma_w / (0.177 + 0.000823 * h)^0.4;
    sigma_v = sigma_u;

    %% Effective airspeed (avoid divide-by-zero)
    V = max(airspeed, 0.5);

    %% Forming filters — discrete-time state-space implementation
    % Dryden transfer functions (continuous-time):
    %   H_u(s) = sigma_u * sqrt(2*Lu/(pi*V)) * 1/(1 + Lu/V * s)
    %   H_v(s) = sigma_v * sqrt(2*Lv/(pi*V)) * (1 + sqrt(3)*Lv/V * s) / (1 + Lv/V * s)^2
    %   H_w(s) = sigma_w * sqrt(2*Lw/(pi*V)) * (1 + sqrt(3)*Lw/V * s) / (1 + Lw/V * s)^2
    %
    % Discretized using bilinear (Tustin) transform for accuracy at all frequencies.

    % White noise inputs
    n_u = randn;
    n_v = randn;
    n_w = randn;

    %% Longitudinal (u) — first-order filter
    tau_u = Lu / V;
    K_u = sigma_u * sqrt(2 * Lu / (pi * V));
    % Bilinear discretization of 1/(1 + tau*s)
    a_u = (2*tau_u - dt) / (2*tau_u + dt);
    b_u = dt / (2*tau_u + dt);
    u_turb = a_u * state_u(1) + b_u * K_u * (n_u + state_u(2));
    state_u = [u_turb; n_u];

    %% Lateral (v) — second-order filter with numerator dynamics
    tau_v = Lv / V;
    K_v = sigma_v * sqrt(2 * Lv / (pi * V));
    beta_v = sqrt(3) * tau_v;
    % State-space representation of (1 + beta*s)/(1 + tau*s)^2
    % Using two cascaded first-order filters with feedforward
    a_v1 = (2*tau_v - dt) / (2*tau_v + dt);
    b_v1 = dt / (2*tau_v + dt);
    % First stage
    y1 = a_v1 * state_v(1) + b_v1 * (n_v + state_v(3));
    % Second stage
    y2 = a_v1 * state_v(2) + b_v1 * (y1 + state_v(1));
    % Feedforward term for the (1 + beta*s) numerator
    v_turb = K_v * (y2 + beta_v / (tau_v + dt/2) * (y1 - state_v(1)));
    state_v = [y1; y2; n_v];

    %% Vertical (w) — second-order filter with numerator dynamics
    tau_w = Lw / V;
    K_w = sigma_w * sqrt(2 * Lw / (pi * V));
    beta_w = sqrt(3) * tau_w;
    a_w1 = (2*tau_w - dt) / (2*tau_w + dt);
    b_w1 = dt / (2*tau_w + dt);
    y1w = a_w1 * state_w(1) + b_w1 * (n_w + state_w(3));
    y2w = a_w1 * state_w(2) + b_w1 * (y1w + state_w(1));
    w_turb = K_w * (y2w + beta_w / (tau_w + dt/2) * (y1w - state_w(1)));
    state_w = [y1w; y2w; n_w];

    %% Assemble output (NED frame — turbulence + steady wind)
    turb_body = [u_turb; v_turb; w_turb];
    wind_body = wind_steady + turb_body;

end
