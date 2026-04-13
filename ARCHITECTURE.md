# SimuLink_College — Drone Simulator Architecture & Methodology

> Comprehensive technical reference for the MATLAB multirotor flight simulator.
> Covers system architecture, mathematical models, control theory, and implementation details.

---

## Table of Contents

1. [System Architecture Overview](#1-system-architecture-overview)
2. [Coordinate Frames & Conventions](#2-coordinate-frames--conventions)
3. [State Vector & Equations of Motion](#3-state-vector--equations-of-motion)
4. [6-DOF Rigid Body Dynamics](#4-6-dof-rigid-body-dynamics)
5. [Propulsion System Model](#5-propulsion-system-model)
6. [Aerodynamic Models](#6-aerodynamic-models)
7. [Environmental Force Models](#7-environmental-force-models)
8. [Battery & Thermal Model](#8-battery--thermal-model)
9. [Cascaded Flight Controller](#9-cascaded-flight-controller)
10. [Manual Flight Controller](#10-manual-flight-controller)
11. [Control Allocation & Mixing](#11-control-allocation--mixing)
12. [Sensor Models](#12-sensor-models)
13. [State Estimation (EKF)](#13-state-estimation-ekf)
14. [Wind & Turbulence Model](#14-wind--turbulence-model)
15. [Navigation & Mission Planning](#15-navigation--mission-planning)
16. [Numerical Integration (RK4)](#16-numerical-integration-rk4)
17. [Configuration & Parameter Derivation](#17-configuration--parameter-derivation)
18. [Visualization & Real-Time Loop](#18-visualization--real-time-loop)
19. [Telemetry & Logging](#19-telemetry--logging)
20. [File Reference](#20-file-reference)

---

## 1. System Architecture Overview

### 1.1 High-Level Block Diagram

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        SIMULATION MAIN LOOP                            │
│                     (live_drone_sim.m / run_simulation.m)              │
│                                                                         │
│  ┌──────────┐   ┌──────────────┐   ┌────────────┐   ┌──────────────┐  │
│  │ Keyboard │──▶│   FLIGHT     │──▶│  CONTROL   │──▶│    MOTOR     │  │
│  │  Input   │   │ CONTROLLER   │   │ ALLOCATION │   │   DYNAMICS   │  │
│  │ (HID)    │   │ (Auto/Manual)│   │ (Mixing)   │   │ (1st-order)  │  │
│  └──────────┘   └──────────────┘   └────────────┘   └──────┬───────┘  │
│                        ▲                                     │          │
│                        │                                     ▼          │
│                 ┌──────┴───────┐                    ┌──────────────┐   │
│                 │    STATE     │◀───────────────────│   6-DOF      │   │
│                 │  ESTIMATOR   │    RK4 Integration │  RIGID BODY  │   │
│                 │   (EKF)      │                    │  DYNAMICS    │   │
│                 └──────┬───────┘                    └──────┬───────┘   │
│                        ▲                                   │           │
│                 ┌──────┴───────┐                           │           │
│                 │   SENSOR     │◀──────────────────────────┘           │
│                 │   MODELS     │     True State + Noise                │
│                 │ IMU/GPS/Baro │                                       │
│                 └──────────────┘                                       │
│                                                                         │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐                 │
│  │  WIND MODEL  │  │   BATTERY    │  │  VIBRATION   │                 │
│  │  (Dryden)    │  │  THERMAL     │  │    MODEL     │                 │
│  └──────────────┘  └──────────────┘  └──────────────┘                 │
└─────────────────────────────────────────────────────────────────────────┘
                              │
                              ▼
        ┌─────────────────────────────────────────┐
        │           VISUALIZATION                  │
        │  ┌──────────┐  ┌──────────┐  ┌────────┐│
        │  │  3D View  │  │   HUD    │  │ Cesium ││
        │  │  (MATLAB) │  │ Overlay  │  │ Globe  ││
        │  └──────────┘  └──────────┘  └────────┘│
        └─────────────────────────────────────────┘
```

### 1.2 Simulation Loop Timing

| Loop           | Rate     | Timestep | Purpose                        |
|----------------|----------|----------|--------------------------------|
| Physics        | 500 Hz   | 2 ms     | RK4 dynamics integration       |
| Motor dynamics | 500 Hz   | 2 ms     | First-order motor response     |
| Rendering      | 40 Hz    | 25 ms    | 3D visualization + HUD         |
| Telemetry log  | 100 Hz   | 10 ms    | Flight data recording          |
| Cesium sync    | 20 Hz    | 50 ms    | External 3D globe update       |

### 1.3 Data Flow

```
User Input ──▶ Controller ──▶ Mixer ──▶ Motors ──▶ Dynamics ──▶ State
     ▲                                                            │
     │                                                            │
     └──────────── Feedback (state for next step) ◀───────────────┘
```

---

## 2. Coordinate Frames & Conventions

### 2.1 NED (North-East-Down) Inertial Frame

The simulation uses the standard aerospace NED convention:

| Axis | Direction | Sign Convention       |
|------|-----------|-----------------------|
| x    | North     | Positive = northward  |
| y    | East      | Positive = eastward   |
| z    | Down      | Positive = downward   |

**Altitude**: $h = -z$ (positive altitude = negative NED z-component)

### 2.2 Body Frame (Forward-Right-Down)

| Axis | Direction | Aligned With                |
|------|-----------|-----------------------------|
| $x_b$ | Forward   | Nose of the drone          |
| $y_b$ | Right     | Starboard side             |
| $z_b$ | Down      | Belly of the drone         |

### 2.3 Euler Angles (ZYX Intrinsic Rotation)

| Symbol   | Name  | Rotation About | Positive Direction |
|----------|-------|----------------|--------------------|
| $\phi$   | Roll  | $x_b$ (body)  | Right wing down    |
| $\theta$ | Pitch | $y_b$ (body)  | Nose up            |
| $\psi$   | Yaw   | $z_b$ (body)  | Nose right (CW from above) |

### 2.4 Rotation Matrix (Body → NED)

The Direction Cosine Matrix (DCM) using ZYX convention:

$$
R_{b}^{n} = R_z(\psi)\, R_y(\theta)\, R_x(\phi)
$$

$$
R_{b}^{n} = \begin{bmatrix}
c_\theta c_\psi & s_\phi s_\theta c_\psi - c_\phi s_\psi & c_\phi s_\theta c_\psi + s_\phi s_\psi \\
c_\theta s_\psi & s_\phi s_\theta s_\psi + c_\phi c_\psi & c_\phi s_\theta s_\psi - s_\phi c_\psi \\
-s_\theta       & s_\phi c_\theta                         & c_\phi c_\theta
\end{bmatrix}
$$

Where $c_x = \cos(x)$, $s_x = \sin(x)$.

### 2.5 Euler Rate Kinematics

Body angular rates $[p, q, r]^T$ map to Euler angle rates:

$$
\begin{bmatrix} \dot\phi \\ \dot\theta \\ \dot\psi \end{bmatrix}
= \begin{bmatrix}
1 & \sin\phi\tan\theta & \cos\phi\tan\theta \\
0 & \cos\phi            & -\sin\phi \\
0 & \sin\phi\sec\theta  & \cos\phi\sec\theta
\end{bmatrix}
\begin{bmatrix} p \\ q \\ r \end{bmatrix}
$$

**Gimbal lock protection**: $\theta$ is clamped to $\pm 80°$ to prevent the $\sec\theta$ singularity at $\pm 90°$.

### 2.6 GPS ↔ NED Conversion (Flat-Earth Approximation)

$$
\text{lat} = \text{lat}_{ref} + \frac{n}{R_{earth}}, \qquad
\text{lon} = \text{lon}_{ref} + \frac{e}{R_{earth} \cos(\text{lat}_{ref})}
$$

Where $R_{earth} = 6{,}378{,}137$ m (WGS-84 equatorial radius).

---

## 3. State Vector & Equations of Motion

### 3.1 State Vector (12-DOF)

$$
\mathbf{x} = \begin{bmatrix} x & y & z & v_x & v_y & v_z & \phi & \theta & \psi & p & q & r \end{bmatrix}^T
$$

| Index  | Symbol     | Description               | Units   |
|--------|------------|---------------------------|---------|
| 1–3    | $x,y,z$    | Position (NED)            | m       |
| 4–6    | $v_x,v_y,v_z$ | Velocity (NED)         | m/s     |
| 7–9    | $\phi,\theta,\psi$ | Euler angles        | rad     |
| 10–12  | $p,q,r$    | Body angular rates        | rad/s   |

### 3.2 State Derivative

$$
\dot{\mathbf{x}} = \begin{bmatrix}
\mathbf{v} \\
\mathbf{F}_{total} / m \\
W(\phi,\theta) \cdot \boldsymbol{\omega}_b \\
I^{-1}\left(\boldsymbol{\tau}_{total} - \boldsymbol{\omega}_b \times I\boldsymbol{\omega}_b\right)
\end{bmatrix}
$$

---

## 4. 6-DOF Rigid Body Dynamics

### 4.1 Translational Dynamics

$$
m\,\dot{\mathbf{v}} = \mathbf{F}_{gravity} + \mathbf{F}_{thrust} + \mathbf{F}_{drag} + \mathbf{F}_{hub} + \mathbf{F}_{ground}
$$

### 4.2 Rotational Dynamics (Euler's Equation)

$$
I\,\dot{\boldsymbol{\omega}} = \boldsymbol{\tau}_{motors} + \boldsymbol{\tau}_{drag} + \boldsymbol{\tau}_{gyro} + \boldsymbol{\tau}_{flap} - \boldsymbol{\omega} \times (I\,\boldsymbol{\omega})
$$

Where $I = \text{diag}(I_{xx}, I_{yy}, I_{zz})$ is the inertia tensor.

### 4.3 Force Models Summary

```
                    ┌─────────────────────────┐
                    │    TOTAL FORCES (NED)    │
                    └─────────┬───────────────┘
           ┌──────────────────┼──────────────────────┐
           │                  │                      │
    ┌──────▼──────┐  ┌───────▼──────┐  ┌────────────▼──────────┐
    │   Gravity   │  │    Thrust    │  │   Aerodynamic Drag    │
    │  [0;0;mg]   │  │  R·[0;0;-T] │  │  -Cd·v·|v|·ρ_ratio   │
    └─────────────┘  └──────┬───────┘  └───────────────────────┘
                            │
              ┌─────────────┼─────────────┐
              │             │             │
     ┌────────▼───┐  ┌─────▼─────┐  ┌────▼────────┐
     │  Density   │  │  Ground   │  │   Vortex    │
     │ Correction │  │  Effect   │  │  Ring State │
     └────────────┘  └───────────┘  └─────────────┘
```

### 4.4 Moment Models Summary

```
                    ┌─────────────────────────┐
                    │   TOTAL MOMENTS (Body)   │
                    └─────────┬───────────────┘
         ┌────────────────────┼────────────────────────┐
         │                    │                        │
  ┌──────▼──────┐   ┌────────▼────────┐   ┌───────────▼──────────┐
  │   Motor     │   │   Rotational    │   │   Gyroscopic         │
  │  Moments    │   │   Drag          │   │   Precession         │
  │ Σ(-yi·Ti)  │   │  -Cd_r·ω·|ω|   │   │  ω × [0;0;h_z·Jp]  │
  └─────────────┘   └─────────────────┘   └──────────────────────┘
                                                     │
                                            ┌────────▼────────┐
                                            │  Blade Flapping  │
                                            │  K_flap·(1+3μ)  │
                                            └─────────────────┘
```

---

## 5. Propulsion System Model

### 5.1 Motor Thrust & Torque

Each motor produces thrust and reaction torque proportional to the square of angular velocity:

$$
T_i = k_T \cdot \omega_i^2, \qquad Q_i = k_Q \cdot \omega_i^2
$$

Where:
- $k_T$ : Thrust coefficient $[\text{N}/(\text{rad/s})^2]$
- $k_Q$ : Torque coefficient $[\text{N·m}/(\text{rad/s})^2]$

### 5.2 Thrust Coefficient Derivation (Blade-Element Momentum Theory)

From propeller geometry:

$$
k_T = \frac{C_T \cdot \rho \cdot D^4}{4\pi^2}, \qquad
k_Q = \frac{C_Q \cdot \rho \cdot D^5}{4\pi^2}
$$

Where $C_T$ and $C_Q$ are empirical coefficients calibrated to the UIUC propeller database:

$$
C_T = 0.0865 \cdot \left(\frac{P}{D}\right)^{0.5} + 0.0275
$$

$$
C_Q = 0.0065 \cdot \left(\frac{P}{D}\right)^{0.7} + 0.0032
$$

$P/D$ is the pitch-to-diameter ratio.

### 5.3 Motor Dynamics (First-Order)

```
   ω_cmd ──▶ [  1/(τ·s + 1)  ] ──▶ ω_out ──▶ T = kT·ω²
```

Discrete-time implementation:

$$
\omega_{out}[k+1] = \omega_{out}[k] + \alpha \cdot (\omega_{cmd} - \omega_{out}[k])
$$

$$
\alpha = \frac{\Delta t}{\tau_{motor} + \Delta t}
$$

**Precise model** (motor_model_precise.m) adds:
- Non-monotonic time constant: $\tau_{eff} = \tau_0 \cdot (1 + 1.2\lambda(1-\lambda))$, peaks at 50% loading
- Back-EMF ceiling: $\omega_{max} = K_v \cdot V_{sag} \cdot 0.95$
- Current limiting per-motor and total pack

### 5.4 Motor Speed Limits

| Parameter     | Formula                                  |
|---------------|------------------------------------------|
| No-load RPM   | $K_v \times V_{battery}$                |
| Loaded max     | $0.85 \times$ No-load RPM              |
| Idle           | $0.10 \times$ No-load RPM              |

### 5.5 Motor Layout (N-Motor Configurations)

Supported frame types and motor placement angles:

| Frame Type | N | Motor Angles (from +x)           | Spin Pattern |
|-----------|---|----------------------------------|--------------|
| tri       | 3 | 30°, 150°, 270°                  | CCW, CW, CCW |
| quad_x    | 4 | 45°, 225°, 315°, 135°            | CW, CW, CCW, CCW |
| quad_+    | 4 | 0°, 180°, 90°, 270°              | CW, CW, CCW, CCW |
| hex_flat  | 6 | 0°, 60°, 120°, 180°, 240°, 300° | Alternating CW/CCW |
| octo_flat | 8 | 0°, 45°, 90°, ...315°            | Alternating CW/CCW |

Motor position from angle and arm length:

$$
x_i = L \cos(\alpha_i), \qquad y_i = L \sin(\alpha_i)
$$

---

## 6. Aerodynamic Models

### 6.1 Translational Drag (Quadratic)

$$
\mathbf{F}_{drag} = -\begin{bmatrix} C_{d,xy} \\ C_{d,xy} \\ C_{d,z} \end{bmatrix} \circ \mathbf{v}_{air} \circ |\mathbf{v}_{air}| \cdot \frac{\rho_{local}}{\rho_{sea}}
$$

Where $\mathbf{v}_{air} = \mathbf{v} - \mathbf{v}_{wind}$ and $\circ$ denotes element-wise multiplication.

**Drag coefficient derivation** (auto mode):

$$
C_{d,xy} = \tfrac{1}{2}\rho \cdot C_d^{bluff} \cdot A_{frontal} \cdot 1.5
$$

$A_{frontal}$ includes body plate, arm cross-sections, and motor housings. The 1.5× interference factor accounts for flow interactions (Hoerner, "Fluid-Dynamic Drag", Ch. 8).

### 6.2 Hub/Parasitic Drag

$$
\mathbf{F}_{hub} = -\frac{1}{2}\rho_{local} \cdot C_d \cdot A_{frontal} \cdot \mathbf{v}_{air} \cdot \|\mathbf{v}_{air}\|
$$

Active only when $\|\mathbf{v}_{air}\|^2 > 0.01$.

### 6.3 Rotational Drag

$$
\boldsymbol{\tau}_{drag} = -C_{d,r} \cdot \boldsymbol{\omega} \circ |\boldsymbol{\omega}|
$$

### 6.4 Altitude Air Density Correction (ISA Atmosphere)

$$
\rho(h) = \rho_0 \cdot (1 - 2.2558 \times 10^{-5} \cdot h)^{4.2559}
$$

Applied to thrust, aerodynamic drag, and hub drag. Floor at $\rho = 0.4$ kg/m³ (~7 km equivalent).

### 6.5 Gyroscopic Precession

Spinning propellers create angular momentum that produces gyroscopic torques during rotation:

$$
h_z = J_p \sum_{i=1}^{N} s_i \cdot \omega_i
$$

$$
\boldsymbol{\tau}_{gyro} = \boldsymbol{\omega}_b \times \begin{bmatrix} 0 \\ 0 \\ h_z \end{bmatrix}
= \begin{bmatrix} q \cdot h_z \\ -p \cdot h_z \\ 0 \end{bmatrix}
$$

Where $J_p = \frac{1}{2} m_{prop} r_{prop}^2$ (thin disc), $s_i = \pm 1$ is spin direction.

### 6.6 Blade Flapping

At forward speed, advancing/retreating blade asymmetry creates roll and pitch moments:

$$
\mu = \frac{\|\mathbf{v}_{body,xy}\|}{V_{tip}} \quad \text{(advance ratio)}
$$

$$
K_{eff} = K_{flap} \cdot (1 + 3\mu) \quad \text{(Prouty, 2002)}
$$

$$
\boldsymbol{\tau}_{flap} = \begin{bmatrix}
-K_{eff} \cdot v_{y,body} \cdot T_{total} \\
+K_{eff} \cdot v_{x,body} \cdot T_{total} \\
0
\end{bmatrix}
$$

### 6.7 Propeller Vibration

Three vibration sources per motor:

| Source | Frequency | Cause | Amplitude |
|--------|-----------|-------|-----------|
| Static imbalance | 1/rev ($\omega_i$) | Prop mass asymmetry | $F = m_e r \omega_i^2$ |
| Dynamic imbalance | 2/rev ($2\omega_i$) | Blade track difference | 10% of 1/rev |
| Structural resonance | Near $f_{nat}$ | Frame bending | Amplified by Q-factor |

Frame natural frequency estimate: $f_{nat} \approx 100 / L^{0.7}$ Hz.

Transmissibility near resonance:

$$
T = \frac{1}{\sqrt{(1 - r_f^2)^2 + (2\zeta r_f)^2}}, \qquad \zeta = \frac{1}{2Q}
$$

---

## 7. Environmental Force Models

### 7.1 Ground Effect (Cheeseman-Bennett, 1955)

When hovering near the ground, the trapped air cushion increases effective thrust:

$$
\frac{T_{GE}}{T_{OGE}} = \frac{1}{1 - \left(\frac{R}{4h}\right)^2}
$$

Where $R$ = rotor radius, $h$ = altitude AGL.

| Condition | Result |
|-----------|--------|
| $h > h_{GE}$ | No effect |
| $h < 0.05$ m | No effect (on ground) |
| $R/(4h) < 1$ | Formula applied |
| $R/(4h) \geq 1$ | Capped at 1.5× |

### 7.2 Ground Contact (Spring-Damper + Friction)

```
  F_normal = -k·δ - c·max(0, vz)     ← Spring-damper (prevents penetration)
  F_friction = -μ·|F_normal|·v̂_h     ← Coulomb friction (opposes sliding)
```

| Parameter | Formula | Typical Value |
|-----------|---------|---------------|
| $k_{ground}$ | $2000 \cdot m$ | 3000 N/m (1.5 kg drone) |
| $c_{ground}$ | $50 \cdot m$ | 75 N·s/m |
| $\mu_{friction}$ | 0.6 | Rubber on concrete |
| Bounce COR | 0.3 | At hard floor |

Static friction engages below 0.05 m/s horizontal velocity.

### 7.3 Vortex Ring State (VRS)

Modeled after Johnson (1980). Occurs during rapid descent through own downwash:

$$
V_i = \sqrt{\frac{T \cdot \rho_{ratio}}{2\rho_{local} A_{disc}}}
\qquad \text{(induced velocity at hover)}
$$

**Onset conditions**: $V_d > 0.3 V_i$ AND $V_h < 2.0 V_i$

**Thrust loss factor**:

$$
f_{VRS} = 1 - 0.6 \cdot \exp\!\left(-4.5\left(\frac{V_d}{V_i} - 1.5\right)^2\right) \cdot \exp\!\left(-2.0\left(\frac{V_h}{V_i}\right)^2\right)
$$

- Peak thrust loss (60%) at $V_d/V_i \approx 1.5$
- Fades rapidly with forward flight speed
- Floor at 40% thrust remaining

---

## 8. Battery & Thermal Model

### 8.1 LiPo Discharge Curve

```
  V_full ─────╮
              ╰──── Gentle decline (80-100% SOC)
  V_nom  ──────────────────── Nearly linear (20-80% SOC)
              ╭──── Steep drop (0-20% SOC)
  V_empty ────╯
```

$$
V_{open}(SOC) = V_{empty} + (V_{full} - V_{empty}) \cdot \left[0.10(1 - e^{-12\cdot SOC}) + 0.76\cdot SOC + 0.14\cdot SOC^4\right]
$$

| Cell Count | $V_{full}$ | $V_{nom}$ | $V_{empty}$ |
|-----------|------------|-----------|-------------|
| 3S        | 12.6 V     | 11.1 V    | 9.9 V       |
| 4S        | 16.8 V     | 14.8 V    | 13.2 V      |
| 6S        | 25.2 V     | 22.2 V    | 19.8 V      |

### 8.2 Voltage Sag Under Load

$$
V_{sag} = \max\!\left(0.9V_{empty},\;\; V_{open} - I_{total} \cdot R_{int}\right)
$$

Internal resistance depends on C-rating:

| C-Rating | $R_{cell}$ (mΩ) |
|----------|-----------------|
| ≥75C     | 8               |
| ≥50C     | 10              |
| ≥25C     | 15              |
| <25C     | 20              |

### 8.3 Motor Efficiency Model

Gaussian bell curve peaking at ~65% throttle:

$$
\eta(\lambda) = \eta_{peak} \cdot \eta_{ESC} \cdot \max\!\left(0.10,\;\; e^{-4.5(\lambda - 0.65)^2}\right)
$$

Where $\lambda = \omega / \omega_{ceiling}$ is throttle fraction.

### 8.4 Battery Thermal Dynamics

$$
\dot{T}_{cell} = \frac{I^2 R_{temp} - h_{conv}(T_{cell} - T_{ambient})}{m_{batt} \cdot C_p \cdot 1.2}
$$

| Parameter | Value | Description |
|-----------|-------|-------------|
| $C_p$ | 1050 J/(kg·K) | LiPo specific heat |
| $h_{conv}$ | 1.2 W/K | Convective cooling (prop wash) |
| $R_{temp}$ | $R_{base}(1 + 0.005\Delta T)$ | Temperature-dependent ESR |

**Thermal protection**:
- Warning at 60°C
- Motor cutoff at 80°C (50% motor authority reduction)

### 8.5 SOC Tracking

$$
SOC = \max\!\left(0,\;\; 1 - \frac{E_{used}}{E_{total}}\right)
$$

$$
E_{used} = \int_0^t V(t') \cdot I(t') \, dt' \quad [\text{Wh}]
$$

---

## 9. Cascaded Flight Controller

### 9.1 Control Architecture

```
 Position    Altitude     Attitude      Rate       Mixing
  (Outer)      (Alt)      (Middle)     (Inner)    (Allocation)
    50 Hz      50 Hz       250 Hz      1000 Hz

  pos_err ──▶ [PID] ──▶ desired ──▶ [PID] ──▶ desired ──▶ [PID] ──▶ τ ──▶ [Mix] ──▶ ω_cmd
              roll/pitch   rates     torques   motor speeds
                   │
  alt_err ──▶ [PID] ──▶ thrust_cmd ──────────────────────────────────────▶ [Mix] ──▶ ω_cmd
```

### 9.2 Position Controller (Outer Loop, 50 Hz)

Produces desired roll and pitch angles from horizontal position error:

$$
\mathbf{a}_{des} = K_p \cdot \mathbf{e}_{xy} + K_i \int \mathbf{e}_{xy}\,dt + K_d \cdot \dot{\mathbf{e}}_{xy}
$$

Convert NED acceleration to body-frame desired attitudes via yaw rotation:

$$
a_{x,body} = \cos\psi \cdot a_{x,NED} + \sin\psi \cdot a_{y,NED}
$$

$$
\theta_{des} = \arctan\!\left(\frac{-a_{x,body}}{g}\right), \qquad
\phi_{des} = \arctan\!\left(\frac{a_{y,body}}{g}\right)
$$

Saturated to $\pm 25°$.

| Gain | Default | Units |
|------|---------|-------|
| $K_p$ | 1.2 | — |
| $K_i$ | 0.05 | — |
| $K_d$ | 0.8 | — |

### 9.3 Altitude Controller (50 Hz)

$$
T_{cmd} = m g - \left(K_p^{alt} \cdot e_z + K_i^{alt} \int e_z\,dt + K_d^{alt}\dot{e}_z\right)
$$

The negative sign before the PID output is because NED z-down: climbing requires more thrust (negative correction).

| Gain | Default | Auto-Tuned |
|------|---------|------------|
| $K_p$ | 4.0 | $3.5 \cdot \min(2, TWR/3)$ |
| $K_i$ | 0.8 | 0.6 |
| $K_d$ | 3.0 | $2\sqrt{K_p \cdot m}$ (critical damping) |

### 9.4 Attitude Controller (Middle Loop, 250 Hz)

PID on attitude error producing desired body angular rates.
Uses **derivative-on-measurement** to avoid setpoint derivative kick:

$$
\mathbf{P} = K_p \cdot \mathbf{e}_{att}
$$

$$
\mathbf{I} = K_i \cdot \text{clamp}\!\left(\int \mathbf{e}_{att}\,dt,\; \pm I_{max}\right)
$$

$$
\mathbf{D} = -K_d \cdot \frac{\Delta\boldsymbol{\phi}_{meas}}{\Delta t}
\qquad\text{(NOT }K_d \cdot \dot{e}\text{)}
$$

Yaw error wrapped to $[-\pi, \pi]$ via $\text{atan2}(\sin e_\psi,\, \cos e_\psi)$.

### 9.5 Rate Controller (Inner Loop, 1000 Hz)

Full PID on angular rate error producing body torque commands:

$$
\boldsymbol{\tau} = K_p \cdot \mathbf{e}_\omega + K_i \int \mathbf{e}_\omega\,dt + K_d \cdot \dot{\mathbf{e}}_\omega
$$

Tight integrator limit: $\pm 0.5$ (prevents windup in fast inner loop).

### 9.6 Auto-Tuning from Plant Parameters

The auto-tuner derives gains from linearized dynamics at hover:

$$
\omega_{n,roll} = \sqrt{\frac{k_T \cdot \omega_{hover} \cdot L \cdot N}{I_{xx}}}
$$

Rate loop bandwidth: $BW_{rate} = \min(50,\;\max(15,\;30\omega_n))$

Rate $K_p = BW \cdot I_{axis}$ (integrator plant → proportional gives bandwidth).

Attitude bandwidth ≈ Rate bandwidth / 4 (cascade separation rule).

---

## 10. Manual Flight Controller

### 10.1 Architecture

```
  Keyboard ──▶ Expo ──▶ ┌───────────────────────────────────┐
   (±1)        Curve    │  Self-Level Mode:                  │
                        │    Sticks ──▶ Desired Attitude     │
                        │    PD Controller ──▶ Moments       │
                        │    Velocity Damping (anti-drift)   │
                        ├───────────────────────────────────┤
                        │  Rate (Acro) Mode:                 │
                        │    Sticks ──▶ Desired Rates        │
                        │    PD Rate Controller ──▶ Moments  │
                        └───────────────────────────────────┘
  SPACE/SHIFT ──▶ Climb Rate Controller ──▶ Thrust Command
```

### 10.2 Expo Curve

Softens stick response near center for precise control:

$$
u_{out} = u_{in} \cdot (1 - e) + u_{in}^3 \cdot e
$$

Where $e \in [0, 1]$ is the expo factor (default 0.35).

### 10.3 Climb Rate Controller

Maps throttle input to desired climb rate, then uses P-controller:

$$
V_{z,des} = u_{throttle} \cdot V_{z,max}
$$

$$
T_{cmd} = \left(mg + (V_{z,des} - V_{z,actual}) \cdot m \cdot K_p\right) \cdot \frac{1}{\cos\phi \cos\theta}
$$

| Parameter | Value |
|-----------|-------|
| $V_{z,max}$ | 5.0 m/s |
| $K_p$ (climb rate) | 3.0 |
| Tilt comp floor | 0.7 (prevents divergence at high bank) |
| Thrust cap | 85% of max thrust |

### 10.4 Self-Level Attitude PD

Gains scale with inertia (normalized to reference 0.0135 $\text{kg·m}^2$):

$$
K_p = 14.0 \cdot \frac{I_{xx}}{0.0135}, \qquad K_d = 4.0 \cdot \frac{I_{xx}}{0.0135}
$$

### 10.5 Velocity Damping (Anti-Drift)

Prevents "ice-skating" in self-level mode by adding opposing torques:

| Condition | Damping Gain |
|-----------|-------------|
| Sticks centered | 3.0 (strong braking) |
| Maneuvering | 0.6 (light) |
| Speed > 70% limit | Progressive ramp-up |

Sign convention:
- Forward velocity ($v_{x,body} > 0$) → pitch-up torque ($+\tau_y$) to brake
- Rightward velocity ($v_{y,body} > 0$) → roll-left torque ($-\tau_x$) to brake

---

## 11. Control Allocation & Mixing

### 11.1 Mixing Matrix

The allocation matrix $A \in \mathbb{R}^{4 \times N}$ maps motor thrusts to the wrench vector:

$$
\begin{bmatrix} T_{total} \\ \tau_x \\ \tau_y \\ \tau_z \end{bmatrix}
= A \cdot \begin{bmatrix} T_1 \\ T_2 \\ \vdots \\ T_N \end{bmatrix}
$$

Where:

$$
A = \begin{bmatrix}
1      & 1      & \cdots & 1 \\
-y_1   & -y_2   & \cdots & -y_N \\
x_1    & x_2    & \cdots & x_N \\
s_1 k_Q/k_T & s_2 k_Q/k_T & \cdots & s_N k_Q/k_T
\end{bmatrix}
$$

### 11.2 Inverse Allocation

Given desired wrench $\mathbf{w} = [T_{cmd};\, \tau_x;\, \tau_y;\, \tau_z]$:

$$
\mathbf{T}_{motors} = \begin{cases}
A^{-1} \mathbf{w} & N = 4 \text{ (square system)} \\
A^\dagger \mathbf{w} & N \neq 4 \text{ (pseudoinverse, minimum-norm)}
\end{cases}
$$

### 11.3 Motor Command Conversion

$$
\omega_{cmd,i} = \sqrt{\frac{\max(T_i, 0)}{k_T}}
$$

Clamped to $[\omega_{min}, \omega_{max}]$.

---

## 12. Sensor Models

### 12.1 Sensor Pipeline

```
  True State ──▶ Sensor Model ──▶ Latency Buffer ──▶ Dropout Filter ──▶ EKF
                 (noise+bias)     (ring buffer)      (random outages)
```

### 12.2 IMU Model

**Accelerometer**:

$$
\mathbf{a}_{meas} = \mathbf{a}_{true} + \mathbf{b}_a + \mathbf{n}_a
$$

$$
\mathbf{b}_a[k+1] = \mathbf{b}_a[k] + \sigma_{bias}\sqrt{\Delta t}\cdot\mathbf{w}_a, \qquad
\mathbf{n}_a = \frac{\sigma_{noise}}{\sqrt{\Delta t}}\cdot\mathbf{w}_a
$$

**Gyroscope**: Same structure with different noise parameters. Saturated to sensor range limits.

### 12.3 GPS Model

- Update rate: 10 Hz (zero-order hold between updates)
- Position noise: $\sigma_{pos} \cdot HDOP$ (horizontal DOP scaling)
- Vertical: 1.5× worse than horizontal
- Velocity noise: additive Gaussian

### 12.4 Barometer Model

- Update rate: 50 Hz
- Slow drift: $\Delta b = \sigma_{drift} \cdot \Delta t$
- White noise: $\sigma_{baro}$

### 12.5 Magnetometer Model

$$
\psi_{mag} = \psi_{true} + \delta_{declination} + \gamma_{hard-iron}(\psi) + n_{mag}
$$

### 12.6 Sensor Latency & Dropout

| Sensor | Latency  | Dropout Probability | Dropout Duration |
|--------|----------|---------------------|------------------|
| IMU    | 1 ms     | 0.01%               | 2 ms             |
| Baro   | 20 ms    | 0.05%               | 100 ms           |
| Mag    | 10 ms    | 0.02%               | 50 ms            |
| GPS    | 100 ms   | 0.1%                | 500 ms           |

---

## 13. State Estimation (EKF)

### 13.1 Extended Kalman Filter

**Estimated state** (12-D):

$$
\hat{\mathbf{x}} = [x,\, y,\, z,\, v_x,\, v_y,\, v_z,\, \phi,\, \theta,\, \psi,\, b_{ax},\, b_{ay},\, b_{az}]^T
$$

### 13.2 Prediction Step

$$
\hat{\mathbf{x}}^- = f(\hat{\mathbf{x}}^+, \mathbf{u})
$$

$$
P^- = F \cdot P^+ \cdot F^T + Q
$$

Process model linearization:
- Position: $\dot{\mathbf{p}} = \mathbf{v}$
- Velocity: $\dot{\mathbf{v}} = R(\phi,\theta,\psi) \cdot (\mathbf{a}_{IMU} - \mathbf{b}_a) + \mathbf{g}$
- Attitude: $\dot{\boldsymbol{\Phi}} = W(\phi,\theta) \cdot \boldsymbol{\omega}_{gyro}$
- Bias: $\dot{\mathbf{b}}_a = 0$ (random walk)

### 13.3 Update Step (Per Sensor)

$$
\mathbf{K} = P^- H^T (H P^- H^T + R)^{-1}
$$

$$
\hat{\mathbf{x}}^+ = \hat{\mathbf{x}}^- + K(\mathbf{z} - H\hat{\mathbf{x}}^-)
$$

$$
P^+ = (I - KH) P^-
$$

**Measurement models**:

| Sensor | $H$ matrix | $R$ diagonal |
|--------|-----------|-------------|
| GPS pos | $[I_3 \;\; 0_{3\times9}]$ | $\sigma_{gps,pos}^2$ |
| GPS vel | $[0_{3\times3} \;\; I_3 \;\; 0_{3\times6}]$ | $\sigma_{gps,vel}^2$ |
| Baro alt | $[0,0,1,0,...,0]$ | $\sigma_{baro}^2$ |
| Mag heading | $[0,...,0,0,0,1,0,0,0]$ | $\sigma_{mag}^2$ |

---

## 14. Wind & Turbulence Model

### 14.1 Dryden Continuous Turbulence Model

Reference: MIL-DTL-9490E §3.7.2, MIL-HDBK-1797 Appendix A

```
  White Noise ──▶ [Dryden Shaping Filter] ──▶ Colored Turbulence
  (randn)         (transfer function)          (realistic PSD)
```

### 14.2 Transfer Functions

**Longitudinal** (1st order):

$$
H_u(s) = \sigma_u \sqrt{\frac{2L_u}{\pi V}} \cdot \frac{1}{1 + \frac{L_u}{V}s}
$$

**Lateral/Vertical** (2nd order):

$$
H_v(s) = \sigma_v \sqrt{\frac{2L_v}{\pi V}} \cdot \frac{1 + \frac{\sqrt{3}L_v}{V}s}{\left(1 + \frac{L_v}{V}s\right)^2}
$$

### 14.3 Scale Lengths (Low Altitude, MIL-HDBK-1797)

$$
L_u = L_v = \frac{h}{(0.177 + 0.000823h)^{1.2}}, \qquad L_w = h
$$

$$
\sigma_u = \sigma_v = \frac{\sigma_w}{(0.177 + 0.000823h)^{0.4}}
$$

Where $h$ = altitude AGL (clamped 3–300 m).

### 14.4 Discretization

Bilinear (Tustin) transform for accuracy at all frequencies:

$$
a = \frac{2\tau - \Delta t}{2\tau + \Delta t}, \qquad
b = \frac{\Delta t}{2\tau + \Delta t}
$$

### 14.5 Turbulence Intensity Presets

| Level    | $\sigma_w$ (m/s) |
|----------|------------------|
| Light    | 0.5              |
| Moderate | 2.0              |
| Severe   | 5.0              |

---

## 15. Navigation & Mission Planning

### 15.1 Waypoint Management

```
  Mission Profile ──▶ Path Planner ──▶ Waypoint Manager ──▶ Flight Controller
  (predefined)        (smoothing)      (sequencing)         (tracking)
```

### 15.2 Waypoint Acceptance Criteria

A waypoint is considered "reached" when:

$$
d_{horiz} = \sqrt{(x - x_{wp})^2 + (y - y_{wp})^2} < r_{accept}
$$

$$
d_{vert} = |z - z_{wp}| < h_{accept}
$$

AND the drone remains within acceptance zone for the loiter time $t_{loiter}$.

| Parameter | Default |
|-----------|---------|
| $r_{accept}$ | 1.5 m |
| $h_{accept}$ | 0.8 m |
| $t_{loiter}$ | 2.0 s |

### 15.3 Path Smoothing Methods

| Method | Order | Continuity | Use Case |
|--------|-------|------------|----------|
| Linear | 1st | $C^0$ (position) | Simple testing |
| Spline (PCHIP) | 3rd | $C^1$ (velocity) | General missions |
| Minimum-snap | 5th | $C^2$ (acceleration) | Smooth camera/cinema |

**Minimum-snap**: Hermite basis functions with 5th-order polynomial segments.

### 15.4 Mission Profiles

| Mission  | Waypoints | Pattern |
|----------|-----------|---------|
| hover    | 1         | Single point at altitude |
| square   | 5         | 4 corners + return to origin |
| circle   | N (36)    | Circular orbit, yaw toward center |
| figure8  | N (36)    | Lissajous: $x = A\sin t$, $y = A\sin 2t$ |
| helix    | N (36×3)  | Ascending spiral, 3 turns |
| landing  | 4         | Hover → descend → low-alt → touchdown |
| survey   | 10        | Lawn-mower (5 parallel legs) |

---

## 16. Numerical Integration (RK4)

### 16.1 4th-Order Runge-Kutta

$$
\mathbf{x}_{n+1} = \mathbf{x}_n + \frac{\Delta t}{6}(\mathbf{k}_1 + 2\mathbf{k}_2 + 2\mathbf{k}_3 + \mathbf{k}_4)
$$

Where:

$$
\begin{aligned}
\mathbf{k}_1 &= f(\mathbf{x}_n,\, \boldsymbol{\omega}_{prev}) \\
\mathbf{k}_2 &= f(\mathbf{x}_n + \tfrac{\Delta t}{2}\mathbf{k}_1,\, \boldsymbol{\omega}_{mid}) \\
\mathbf{k}_3 &= f(\mathbf{x}_n + \tfrac{\Delta t}{2}\mathbf{k}_2,\, \boldsymbol{\omega}_{mid}) \\
\mathbf{k}_4 &= f(\mathbf{x}_n + \Delta t\,\mathbf{k}_3,\, \boldsymbol{\omega}_{new})
\end{aligned}
$$

Motor speeds are **interpolated** across RK4 substeps:
- $\boldsymbol{\omega}_{prev}$ = motor speeds at start of timestep
- $\boldsymbol{\omega}_{mid} = \frac{1}{2}(\boldsymbol{\omega}_{prev} + \boldsymbol{\omega}_{new})$
- $\boldsymbol{\omega}_{new}$ = motor speeds after dynamics update

### 16.2 Post-Integration Safety Clamps

| Quantity | Limit | Action |
|----------|-------|--------|
| Yaw angle | $[-\pi, \pi]$ | `atan2(sin, cos)` wrapping |
| Roll/Pitch | $\pm 45°$ | Hard clamp + 50% rate damping |
| Yaw rate | $\pm 360°$/s | Hard clamp |
| Horizontal speed | `max_speed` | Normalize velocity vector |
| Vertical speed | $2.5 \times V_{z,max}$ | Clamp |
| Altitude | $\geq 0$ | Floor with bounce (COR=0.3) |

---

## 17. Configuration & Parameter Derivation

### 17.1 Drone Presets

| Preset | Frame | N | Arm [mm] | Mass [kg] | Props | Motor Kv | Battery |
|--------|-------|---|----------|-----------|-------|----------|---------|
| mini_quad | quad_x | 4 | 125 | 0.6 | 5×3" | 2300 | 4S 1.3Ah 75C |
| standard_quad | quad_x | 4 | 230 | 1.5 | 10×4.5" | 920 | 4S 5.0Ah 25C |
| heavy_hex | hex_flat | 6 | 340 | 4.2 | 15×5.5" | 580 | 6S 10Ah 25C |
| octo_lift | octo_flat | 8 | 500 | 8.0 | 18×6" | 380 | 6S 16Ah 25C |
| micro_tri | tri | 3 | 100 | 0.35 | 5×2.5" | 2400 | 3S 0.85Ah 45C |

### 17.2 Inertia Estimation Pipeline

$$
I_{total} = I_{motors} + I_{props} + I_{body} + I_{arms} + I_{battery}
$$

Each component uses the parallel axis theorem:

$$
I_{xx} = \sum_{i=1}^N m_i y_i^2 + \sum I_{self,i}
$$

**Motor self-inertia** (hollow cylinder, outrunner):

$$
I_{transverse} = \frac{m}{12}\left(3(r_{out}^2 + r_{in}^2) + h^2\right), \qquad r_{in} = 0.65\, r_{out}
$$

**Prop disc inertia** (thin disc):

$$
I_{z,prop} = \frac{1}{2}m_{prop}r^2, \qquad I_{x,prop} = I_{y,prop} = \frac{1}{4}m_{prop}r^2
$$

**Central body** (cuboid):

$$
I_{xx,body} = \frac{m_{body}}{12}(d^2 + h^2)
$$

**Arms** (slender rod pivoting at one end):

$$
I_{arm} = \frac{m_{arm} L_{arm}^2}{3}
$$

### 17.3 Performance Estimates

| Metric | Formula |
|--------|---------|
| Hover speed | $\omega_h = \sqrt{mg/(N \cdot k_T)}$ |
| Max thrust | $N \cdot k_T \cdot \omega_{max}^2$ |
| Thrust-to-weight | $T_{max} / (mg)$ |
| Hover power | $N \cdot k_Q \cdot \omega_h^3 / (\eta_m \cdot \eta_{ESC})$ |
| Flight time | $0.8 \cdot C_{Ah} \cdot V_{batt} / P_{hover} \times 60$ min |
| Max speed | $\sqrt{T_{max}\sin 30° / C_{d,total}}$ |
| Max climb | $(T_{max} - mg) / mg \times 5$ m/s |

---

## 18. Visualization & Real-Time Loop

### 18.1 Camera Modes

| Mode | Name | Behavior |
|------|------|----------|
| 1 | Chase | Follows behind drone, smoothed heading |
| 2 | FPV | First-person: camera at drone, looking forward |
| 3 | Orbit | Auto-rotates around drone at fixed distance |
| 4 | Cinematic | Slow-tracking, wide angle |
| 5 | Street | Ground-level pedestrian view |

All modes support scroll-wheel zoom ($0.2\times$ to $5.0\times$ distance).

### 18.2 HUD Pages

| Page | Contents |
|------|----------|
| Flight | Altitude, speed, climb rate, heading, attitude, battery, GPS coords |
| Config | Drone specs, frame type, motor count, thrust-to-weight |
| Performance | Max altitude, max speed, G-force, distance, flight time, efficiency |

### 18.3 3D Environment

```
  ┌────────────────────────────────────────┐
  │              Sky Dome                   │
  │  ┌──────────────────────────────────┐  │
  │  │        3D Procedural City        │  │
  │  │  ┌────────────────────────────┐  │  │
  │  │  │   Dynamic Traffic (cars)   │  │  │
  │  │  │  ┌──────────────────────┐  │  │  │
  │  │  │  │   OSM Ground Tiles   │  │  │  │
  │  │  │  │  (OpenStreetMap)      │  │  │  │
  │  │  │  └──────────────────────┘  │  │  │
  │  │  └────────────────────────────┘  │  │
  │  └──────────────────────────────────┘  │
  └────────────────────────────────────────┘
```

### 18.4 Cesium 3D Globe (Optional)

- CesiumJS 1.107 via Python HTTP server
- Google Photorealistic 3D Tiles (asset 2275207)
- OSM Buildings fallback (asset 96188)
- 20 Hz state polling, drone entity with trail
- Toggle with `V` key

### 18.5 Location Presets

| Location | Latitude | Longitude |
|----------|----------|-----------|
| Midtown NYC | 40.7580 | -73.9855 |
| San Francisco | 37.7749 | -122.4194 |
| London | 51.5074 | -0.1278 |
| Tokyo | 35.6762 | 139.6503 |
| Dubai | 25.2048 | 55.2708 |
| Bengaluru | 12.9716 | 77.5946 |

---

## 19. Telemetry & Logging

### 19.1 Recorded Data Channels

| Category | Channels |
|----------|----------|
| Kinematics | position (3), velocity (3), euler (3), omega (3) |
| Control | thrust_cmd, moment_cmds (3), motor_speeds (N) |
| Tracking | pos_error (3), att_error (3), desired_euler (3) |
| Sensors | gps_pos (3), baro_alt, est_pos (3) |
| Status | battery_soc, wp_index, wp_distance, wind (3) |

### 19.2 Post-Flight Analysis

Generated figures:
1. 3D flight path trajectory
2. Position vs time (X, Y, altitude)
3. Velocity vs time (Vx, Vy, Vz, speed)
4. Attitude history (roll, pitch, yaw)
5. Control commands (thrust, moments)
6. Motor speeds (RPM per motor)
7. Position tracking error (with RMS)
8. Battery SOC discharge curve

---

## 20. File Reference

### 20.1 Directory Structure

```
SimuLink_College/
├── params/
│   ├── drone_config.m          ← Master config builder (5 presets + manual + auto)
│   ├── drone_params.m          ← Legacy default parameters
│   ├── controller_params.m     ← PID gains & rate limits
│   ├── sim_params.m            ← Timing & scenario defaults
│   └── sensor_params.m         ← Sensor noise/bias parameters
│
├── models/
│   ├── multirotor_dynamics.m   ← Generalized N-motor 6-DOF dynamics
│   ├── quadrotor_dynamics.m    ← Legacy 4-motor dynamics
│   ├── motor_model.m           ← Basic 1st-order motor model
│   ├── motor_model_precise.m   ← High-fidelity motor + battery model
│   ├── mixing_matrix.m         ← Legacy 4-motor mixing
│   ├── mixing_matrix_n.m       ← N-motor pseudoinverse allocation
│   ├── propeller_vibration_model.m  ← 1/rev + 2/rev vibration
│   ├── battery_thermal_model.m ← I²R heating + convective cooling
│   └── build_drone_simulink.m  ← Simulink model builder
│
├── control/
│   ├── flight_controller.m     ← Full cascaded controller (standalone)
│   ├── position_controller.m   ← Outer loop: pos → desired attitude
│   ├── altitude_controller.m   ← Alt PID → thrust command
│   └── attitude_controller.m   ← Middle loop: att → desired rates (D-on-measurement)
│
├── environment/
│   └── dryden_wind_model.m     ← MIL-HDBK-1797 Dryden turbulence
│
├── sensors/
│   ├── imu_model.m             ← Accelerometer + gyroscope noise/bias
│   ├── gps_model.m             ← GPS position/velocity with HDOP
│   ├── barometer_model.m       ← Baro altimeter with drift
│   ├── magnetometer_model.m    ← Heading with declination + hard-iron
│   ├── sensor_latency_model.m  ← Ring buffer delays + dropouts
│   └── state_estimator.m       ← 12-state EKF fusion
│
├── navigation/
│   ├── waypoint_manager.m      ← Waypoint sequencing + loiter
│   ├── path_planner.m          ← Linear / spline / min-snap smoothing
│   └── mission_profiles.m      ← 7 predefined mission templates
│
├── telemetry/
│   ├── telemetry_logger.m      ← Flight data recorder (class)
│   ├── telemetry_dashboard.m   ← Real-time multi-plot display
│   └── post_flight_analysis.m  ← Post-flight figures & statistics
│
├── utilities/
│   ├── coord_transforms.m      ← NED↔Body, NED↔LLA (WGS-84)
│   ├── rotation_utils.m        ← DCM, axis-angle, individual rotations
│   └── quaternion_utils.m      ← Quaternion math (scalar-first [w,x,y,z])
│
├── visualization/
│   ├── live_drone_sim.m        ← Main interactive 3D simulator (~2500 lines)
│   ├── drone_sim_launcher.m    ← Pre-flight GUI (dark theme, presets)
│   ├── cesium_viewer.html      ← CesiumJS 3D globe page
│   ├── cesium_server.py        ← Python HTTP server for Cesium
│   └── cesium_bridge.m         ← MATLAB ↔ Cesium state bridge
│
├── tests/
│   └── run_all_tests.m         ← 53 tests (config, motor, dynamics, wind, hover)
│
├── run_simulation.m            ← Headless batch simulation entry point
├── fly_drone.m                 ← Interactive launcher with menu/presets
├── init_project.m              ← Path setup + welcome message
└── PHYSICS_GUIDE.md            ← Physics reference (original)
```

### 20.2 Entry Points

| Command | Mode | Description |
|---------|------|-------------|
| `drone_sim_launcher` | GUI | Pre-flight config UI → `live_drone_sim` |
| `live_drone_sim(cfg)` | Interactive | Real-time 3D flight with keyboard |
| `run_simulation('circle')` | Batch | Autonomous mission with telemetry |
| `fly_drone('menu')` | CLI | Interactive text menu launcher |
| `fly_drone('quick')` | Quick | Instant launch with defaults |

---

*Document generated from codebase audit — April 2026*
