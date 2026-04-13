function batt = battery_thermal_model(batt, current, dt, dp)
% BATTERY_THERMAL_MODEL  Simulates LiPo battery temperature dynamics.
%
%   batt = battery_thermal_model(batt, current, dt, dp)
%
%   Models heat generation from internal resistance (I²R losses) and
%   convective cooling. Implements thermal cutoff protection and
%   voltage derating with temperature.
%
%   Inputs:
%     batt    - battery state struct with fields:
%                .temperature  - current cell temperature [°C]
%                .voltage      - current voltage [V]
%     current - total pack current draw [A]
%     dt      - time step [s]
%     dp      - drone config struct
%
%   Outputs:
%     batt - updated battery state with new temperature and thermal flags
%
%   Temperature effects modeled:
%     1) I²R heating from internal resistance
%     2) Convective cooling to ambient air
%     3) Voltage derating at high temperature (~0.3% per °C above 45°C)
%     4) Internal resistance increase with temperature
%     5) Thermal cutoff warning at 60°C, shutdown at 80°C
%
%   Reference:
%     Tremblay et al. (2007), "A Generic Battery Model for Dynamic Simulation"
%     Berrueta et al. (2018), "Combined Dynamic Programming and Region-Elimination"

    %% Initialize thermal state if needed
    if ~isfield(batt, 'temperature')
        batt.temperature = 25.0;  % Ambient start [°C]
    end
    if ~isfield(batt, 'thermal_warning')
        batt.thermal_warning = false;
    end
    if ~isfield(batt, 'thermal_cutoff')
        batt.thermal_cutoff = false;
    end
    if ~isfield(batt, 'motor_thermal')
        batt.motor_thermal = 25.0;  % Motor winding temperature [°C]
    end

    %% Battery parameters
    T_ambient = 25.0;  % Ambient temperature [°C]

    % Thermal mass: typical LiPo cell ~1050 J/(kg·K), housing adds ~20%
    % For a typical 5Ah pack: mass ≈ 0.3-0.5 kg
    if isfield(dp, 'battery_mass')
        mass_batt = dp.battery_mass;
    else
        mass_batt = dp.energy_Wh / 200;  % Rough estimate: ~200 Wh/kg for LiPo
    end
    Cp_batt = 1050;  % Specific heat [J/(kg·K)]
    thermal_mass = mass_batt * Cp_batt * 1.2;  % +20% for housing

    % Convective cooling coefficient [W/K]
    % Natural convection + prop wash: h*A ≈ 0.5-2.0 W/K for typical drone battery
    h_conv = 1.2;  % Effective convective heat transfer coefficient

    %% Internal resistance temperature dependence
    % LiPo R_internal increases ~0.5% per °C above 25°C (Arrhenius-type)
    R_base = dp.R_internal;
    T_cell = batt.temperature;
    R_temp = R_base * (1 + 0.005 * max(0, T_cell - 25));

    %% Heat generation: I²R losses
    Q_gen = current^2 * R_temp;  % [W]

    %% Heat dissipation: convective cooling
    Q_cool = h_conv * (T_cell - T_ambient);  % [W]

    %% Temperature update (first-order thermal dynamics)
    dT = (Q_gen - Q_cool) * dt / thermal_mass;
    batt.temperature = T_cell + dT;

    %% Motor thermal model (simplified)
    % Motor windings heat up faster (lower thermal mass) but cool faster (metal + prop wash)
    motor_R_loss = 0.3 * Q_gen;  % ~30% of total losses are in motor windings
    motor_thermal_mass = 0.05 * Cp_batt;  % Much smaller than battery
    motor_h_conv = 3.0;  % Better cooling from prop wash
    dT_motor = (motor_R_loss - motor_h_conv * (batt.motor_thermal - T_ambient)) ...
               * dt / motor_thermal_mass;
    batt.motor_thermal = batt.motor_thermal + dT_motor;

    %% Thermal protection
    if batt.temperature > 60
        batt.thermal_warning = true;
    else
        batt.thermal_warning = false;
    end

    if batt.temperature > 80
        batt.thermal_cutoff = true;  % Should trigger motor authority reduction
    else
        batt.thermal_cutoff = false;
    end

    %% Voltage derating at high temperature
    % Above 45°C, voltage drops ~0.3% per degree due to accelerated
    % chemical degradation and increased self-discharge
    if batt.temperature > 45
        temp_derate = 1.0 - 0.003 * (batt.temperature - 45);
        temp_derate = max(0.85, temp_derate);  % Max 15% derate
        batt.voltage = batt.voltage * temp_derate;
    end

    %% Motor efficiency derating from winding temperature
    % Motor copper resistance increases ~0.4% per °C above 25°C
    % This reduces efficiency at high motor temperatures
    if batt.motor_thermal > 40
        batt.motor_eff_derate = 1.0 - 0.004 * (batt.motor_thermal - 25);
        batt.motor_eff_derate = max(0.80, batt.motor_eff_derate);
    else
        batt.motor_eff_derate = 1.0;
    end

end
