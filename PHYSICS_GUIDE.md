# Physics Model Documentation

## Drone Simulation — Complete Mathematical Reference

This document provides the mathematical derivations and physical rationale for every model
in the simulation. All equations use SI units unless noted otherwise.

---

## 1. Coordinate Systems

### 1.1 Reference Frames
- **NED (North-East-Down)**: Inertial frame. Origin at launch point. x=North, y=East, z=Down.
- **Body Frame**: Fixed to drone center of mass. x=Forward, y=Right, z=Down.
- **Propeller Frame**: Fixed to each motor mount, aligned with motor shaft (z = thrust axis).

### 1.2 Rotation Convention
ZYX Euler angles (Tait-Bryan): Yaw (ψ) → Pitch (θ) → Roll (φ)

Rotation matrix (Body → NED):
```
R = Rz(ψ) · Ry(θ) · Rx(φ)

    ⎡ cθcψ   sφsθcψ−cφsψ   cφsθcψ+sφsψ ⎤
R = ⎢ cθsψ   sφsθsψ+cφcψ   cφsθsψ−sφcψ ⎥
    ⎣ −sθ    sφcθ           cφcθ          ⎦
```

### 1.3 Euler Rate Kinematic Equation
Maps body angular rates [p, q, r] to Euler angle rates [φ̇, θ̇, ψ̇]:
```
⎡φ̇⎤   ⎡1  sφtθ   cφtθ ⎤ ⎡p⎤
⎢θ̇⎥ = ⎢0  cφ     −sφ  ⎥ ⎢q⎥
⎣ψ̇⎦   ⎣0  sφ/cθ  cφ/cθ⎦ ⎣r⎦
```

**Gimbal Lock**: Singular at θ = ±90° (tan(θ) → ∞). Protected by clamping θ to ±80°.

---

## 2. Rigid Body Dynamics

### 2.1 Translational Dynamics (NED Frame)
Newton's second law:
```
m · v̇ = F_gravity + F_thrust + F_drag + F_hub_drag + F_ground_effect
```

Where:
- `F_gravity = [0, 0, m·g]ᵀ` (NED, positive z = down)
- `F_thrust = R · [0, 0, −ΣTᵢ]ᵀ` (body thrust rotated to NED)
- `F_drag = −[Cd_xy, Cd_xy, Cd_z]ᵀ ⊙ v_air ⊙ |v_air|` (quadratic drag)

### 2.2 Rotational Dynamics (Body Frame)
Euler's rotation equation:
```
I · ω̇ = τ_motors + τ_drag + τ_gyro + τ_flap − ω × (I · ω)
```

Where `I` is the 3×3 inertia tensor and `ω × (I·ω)` is the Coriolis coupling term.

### 2.3 Inertia Tensor
Computed from component geometry (cuboid body + rod arms + point-mass motors):
```
I_body = diag([(m_b/12)(h²+d²), (m_b/12)(h²+w²), (m_b/12)(w²+d²)])
I_arm  = m_arm · L²/3  (rod pivoting at one end, parallel axis theorem)
I_motor = 0.5 · m_motor · (r_out² + r_in²)  (hollow cylinder, outrunner)
```

---

## 3. Motor Model

### 3.1 Thrust and Torque
Blade Element Momentum Theory (BEMT):
```
T = kT · ω²    [N]
Q = kQ · ω²    [N·m]
```

Where:
- `kT = CT · ρ · D⁴`  (thrust coefficient from propeller data)
- `kQ = CQ · ρ · D⁵`  (torque coefficient from propeller data)
- `CT ≈ 0.11`, `CQ ≈ 0.0045` (calibrated to UIUC propeller database)

### 3.2 Motor Electrical Model
Brushless DC motor with back-EMF:
```
V_motor = Kv⁻¹ · ω + I · R_winding
ω_max = Kv · V_supply · η_ESC
```

### 3.3 Efficiency Model
Gaussian bell-curve peaked at 65% throttle:
```
η = η_peak · η_ESC · max(0.10, exp(−4.5 · (throttle − 0.65)²))
```

Matches measured curves from T-Motor U-series and KDE Direct motors.

### 3.4 Current Limiting
- Per-motor thermal limit: `I_max = 1.5 · I_pack_max / N_motors`
- Total pack limit: `ΣI_motors ≤ I_pack_max = C_rating · Capacity`

---

## 4. Battery Model

### 4.1 LiPo Discharge Curve
Open-circuit voltage vs State of Charge (empirical polynomial):
```
V_oc = V_empty + (V_full − V_empty) · [0.10·(1−e^(−12·SOC)) + 0.76·SOC + 0.14·SOC⁴]
```

Three regions:
1. SOC 80-100%: Gentle decline (fully charged plateau)
2. SOC 20-80%: Nearly linear (bulk energy delivery)
3. SOC 0-20%: Steep voltage drop (electrolyte depletion)

### 4.2 Internal Resistance
Temperature-dependent:
```
R(T) = R_base · (1 + 0.005 · max(0, T − 25°C))
```

### 4.3 Thermal Model
First-order lumped thermal dynamics:
```
C_thermal · dT/dt = I²R − h_conv · (T − T_ambient)
```
Where C_thermal = 1.2 · m_battery · 1050 J/(kg·K)

---

## 5. Aerodynamic Effects

### 5.1 Ground Effect (Cheeseman & Bennett, 1955)
Thrust augmentation near the ground:
```
T_GE / T_OGE = 1 / (1 − (R/(4h))²)
```
- R = rotor radius [m]
- h = height above ground [m]
- Valid for h > R/4; saturated at 1.5× maximum

Reference: Cheeseman, I.C. and Bennett, W.E. (1955). "The Effect of the Ground on a
Helicopter Rotor in Forward Flight." ARC R&M 3021.

### 5.2 Blade Flapping
At forward speed, advancing/retreating blade asymmetry creates rolling/pitching moments:
```
τ_flap = K_flap · (1 + 3μ) · T_total · [−v_y; v_x; 0]
```
Where μ = V_forward / V_tip is the advance ratio.

Reference: Prouty, R.W. (2002). "Helicopter Performance, Stability, and Control."

### 5.3 Gyroscopic Precession
Spinning propellers resist angular rate changes:
```
τ_gyro = ω_body × [0; 0; Σ(sᵢ · Jp · ωᵢ)]
```
Where sᵢ is the spin direction (+1 CCW, −1 CW) and Jp is propeller polar MoI.

### 5.4 Hub Drag
Parasitic drag from motor pods and frame in forward flight:
```
F_hub = −0.5 · ρ · Cd_hub · A_frontal · v · |v|
```

---

## 6. Wind Model (MIL-DTL-9490E Dryden Spectrum)

### 6.1 Turbulence Power Spectral Density
The Dryden model generates colored noise matching atmospheric turbulence:
```
Φ_u(Ω) = σ_u² · (2L_u / π) / (1 + (L_u · Ω)²)
Φ_v(Ω) = σ_v² · (L_v / π) · (1 + 3(L_v · Ω)²) / (1 + (L_v · Ω)²)²
Φ_w(Ω) = σ_w² · (L_w / π) · (1 + 3(L_w · Ω)²) / (1 + (L_w · Ω)²)²
```

### 6.2 Scale Lengths (Low Altitude, h < 300m)
```
L_u = L_v = h / (0.177 + 0.000823h)^1.2
L_w = h
```

### 6.3 Turbulence Intensities
```
Light:    σ_w = 0.5 m/s
Moderate: σ_w = 2.0 m/s
Severe:   σ_w = 5.0 m/s
```

Reference: MIL-HDBK-1797, Appendix A, Section A.8.12.

---

## 7. Sensor Models

### 7.1 IMU (Accelerometer + Gyroscope)
```
measurement = truth + bias + white_noise
bias(k+1) = bias(k) + bias_stability · √dt · N(0,1)
noise = noise_density · √(1/dt) · N(0,1)
```
Saturation: ±16g (accel), ±2000°/s (gyro)

### 7.2 GPS
- Update rate: 5 Hz with 100ms latency
- Position noise: σ = 1.5m · HDOP
- Dropout simulation: random 500ms outages

### 7.3 Barometer
- Update rate: 50 Hz with 20ms latency
- Altitude noise: σ = 0.5m
- Temperature drift: 0.02 m/°C

### 7.4 Magnetometer
- Update rate: 100 Hz
- Hard iron offset + soft iron distortion
- Magnetic declination compensation

---

## 8. Control Architecture

### 8.1 Cascaded PID Control
```
Position PID (50 Hz)  →  desired roll/pitch
Attitude PID (250 Hz) →  desired angular rates
Rate PID (1000 Hz)    →  torque commands
Mixing Matrix         →  individual motor speeds
```

### 8.2 Position Controller
Converts position error to desired tilt angles:
```
a_desired = Kp·e + Ki·∫e·dt + Kd·ė    (NED X-Y)
θ_desired = atan2(a_body_x, g)
φ_desired = atan2(−a_body_y, g)
```

### 8.3 Altitude Controller
PID with gravity feedforward:
```
T_cmd = m·g − (Kp·e_z + Ki·∫e_z·dt + Kd·ė_z)
```

### 8.4 Rate Controller
Full PID (P+I+D) on angular rates with tight anti-windup:
```
τ = Kp·e_ω + Ki·∫e_ω·dt + Kd·ė_ω
```

---

## 9. State Estimation (EKF)

### 9.1 State Vector
12 states: [x, y, z, vx, vy, vz, φ, θ, ψ, p, q, r]

### 9.2 Prediction Step
Uses rigid body dynamics equations with IMU measurements as inputs.

### 9.3 Update Step
Fuses GPS (position + velocity), barometer (altitude), and magnetometer (heading)
using standard Kalman update equations with measurement-specific noise matrices.

---

## 10. Numerical Integration

### 10.1 RK4 (4th-order Runge-Kutta)
```
k1 = f(t, y)
k2 = f(t + dt/2, y + dt/2 · k1)
k3 = f(t + dt/2, y + dt/2 · k2)
k4 = f(t + dt, y + dt · k3)
y(t+dt) = y(t) + (dt/6)(k1 + 2k2 + 2k3 + k4)
```

Time step: dt = 0.002s (500 Hz) — chosen for stability with motor time constants (~20ms).

### 10.2 Sub-stepping
Variable render rate with fixed physics rate ensures consistent dynamics regardless
of display performance.

---

## 11. Vibration Model

### 11.1 Static Imbalance (1/rev)
Centrifugal force from mass asymmetry:
```
F = m_e · r · ω²
```
Rotates at motor frequency, projects into body X-Y plane.

### 11.2 Dynamic Imbalance (2/rev)
Blade track difference creates vertical oscillation at 2× motor frequency.
Amplitude ≈ 10% of static imbalance.

### 11.3 Structural Resonance
Transmissibility amplification near frame natural frequency:
```
T(r) = 1 / √((1−r²)² + (2ζr)²)
```
Where r = f_motor / f_natural, ζ = 1/(2Q), Q ≈ 5 for carbon fiber.

---

## 12. References

1. Cheeseman, I.C. & Bennett, W.E. (1955). ARC R&M 3021.
2. Leishman, J.G. (2006). "Principles of Helicopter Aerodynamics." Cambridge UP.
3. Prouty, R.W. (2002). "Helicopter Performance, Stability, and Control." Krieger.
4. Pounds, P. et al. (2010). "Modelling and control of a large quadrotor robot." CEP.
5. MIL-HDBK-1797 (1997). "Flying Qualities of Piloted Aircraft."
6. MIL-DTL-9490E (2008). "Flight Control Systems."
7. Tremblay, O. et al. (2007). "A Generic Battery Model." IEEE VPPC.
8. UIUC Propeller Database. (https://m-selig.ae.illinois.edu/props/propDB.html)
9. Hoerner, S.F. (1965). "Fluid-Dynamic Drag." Published by author.
10. Berrueta, A. et al. (2018). "Combined Dynamic Programming." Applied Energy.
