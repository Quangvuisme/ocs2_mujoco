# 02. Mathematical Foundations

## 📐 Centroidal Dynamics Model

### What is Centroidal Dynamics?

The **Centroidal Dynamics** model describes the motion of a robot in terms of its **Center of Mass (CoM)** and **total momentum**, rather than tracking every link individually.

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                     CENTROIDAL DYNAMICS CONCEPT                             │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│        Full Rigid Body Model            Centroidal Model                    │
│        (High Dimensional)               (Reduced Order)                     │
│                                                                             │
│        ┌───┐                            ┌──────────────────┐               │
│       /│   │\                           │    CoM Point     │               │
│      ┌─┘   └─┐                          │        ●         │               │
│     /│       │\                         │   Momentum       │               │
│   ┌──┘       └──┐                       │   h = [p, L]     │               │
│   │   Robot    │                        │                  │               │
│   └──┬─────┬──┘                         │   Equations:     │               │
│      │     │                            │   ṗ = mg + ΣF    │               │
│     ┌┴┐   ┌┴┐                           │   L̇ = Σ(r×F)     │               │
│     │ │   │ │                           │                  │               │
│     └─┘   └─┘                           └──────────────────┘               │
│                                                                             │
│   DOF: 6 + n_joints                     State: 12 (momentum + pose)         │
│   + joint dynamics                      + joint kinematics                  │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Centroidal Momentum

The **centroidal momentum** $h$ consists of:

$$h = \begin{bmatrix} p \\ L \end{bmatrix} = A_G(q) \dot{q}$$

Where:
- $p \in \mathbb{R}^3$: Linear momentum at CoM
- $L \in \mathbb{R}^3$: Angular momentum about CoM
- $A_G(q) \in \mathbb{R}^{6 \times n}$: Centroidal Momentum Matrix (CMM)
- $q$: Generalized coordinates
- $\dot{q}$: Generalized velocities

### Normalized Centroidal Momentum

OCS2 uses **normalized** momentum (divided by robot mass):

$$\tilde{h} = \frac{h}{m} = \begin{bmatrix} v_{com} \\ L/m \end{bmatrix}$$

This gives:
- $v_{com} = \dot{p}_{com}$: CoM velocity [m/s]
- $L/m$: Normalized angular momentum [m²/s]

---

## 📊 State and Input Definitions

### State Vector (24-dim)

$$x = \begin{bmatrix} \tilde{h} \\ q_{base} \\ q_{joints} \end{bmatrix} \in \mathbb{R}^{24}$$

| Index | Symbol | Description | Units |
|-------|--------|-------------|-------|
| 0-2 | $v_{com}$ | CoM linear velocity | m/s |
| 3-5 | $L/m$ | Normalized angular momentum | m²/s |
| 6-8 | $p_{base}$ | Base position (x, y, z) | m |
| 9-11 | $\theta_{base}$ | Base orientation (yaw, pitch, roll) | rad |
| 12-14 | $q_{LF}$ | LF joint angles (HAA, HFE, KFE) | rad |
| 15-17 | $q_{LH}$ | LH joint angles | rad |
| 18-20 | $q_{RF}$ | RF joint angles | rad |
| 21-23 | $q_{RH}$ | RH joint angles | rad |

### Input Vector (24-dim)

$$u = \begin{bmatrix} F_1 \\ F_2 \\ F_3 \\ F_4 \\ v_{foot,1} \\ v_{foot,2} \\ v_{foot,3} \\ v_{foot,4} \end{bmatrix} \in \mathbb{R}^{24}$$

| Index | Symbol | Description | Units |
|-------|--------|-------------|-------|
| 0-2 | $F_{LF}$ | LF contact force (Fx, Fy, Fz) | N |
| 3-5 | $F_{RF}$ | RF contact force | N |
| 6-8 | $F_{LH}$ | LH contact force | N |
| 9-11 | $F_{RH}$ | RH contact force | N |
| 12-14 | $v_{LF}$ | LF foot velocity | m/s |
| 15-17 | $v_{LH}$ | LH foot velocity | m/s |
| 18-20 | $v_{RF}$ | RF foot velocity | m/s |
| 21-23 | $v_{RH}$ | RH foot velocity | m/s |

---

## ⚙️ Dynamics Equations

### Centroidal Momentum Dynamics

The rate of change of centroidal momentum equals the sum of external forces and moments:

$$\dot{h} = \begin{bmatrix} \dot{p} \\ \dot{L} \end{bmatrix} = \begin{bmatrix} mg + \sum_{i=1}^{n_c} F_i \\ \sum_{i=1}^{n_c} (r_i - p_{com}) \times F_i \end{bmatrix}$$

Where:
- $m = 52.1348$ kg (ANYmal C mass)
- $g = [0, 0, -9.81]^T$ m/s² (gravity)
- $F_i$: Contact force at foot $i$
- $r_i$: Position of foot $i$ in world frame
- $p_{com}$: CoM position
- $n_c = 4$: Number of contact points

### Single Rigid Body Dynamics (SRBD)

When `centroidalModelType = 1` (used in this project), the robot is approximated as a **Single Rigid Body**:

$$\begin{bmatrix} m\ddot{p}_{com} \\ I_G \dot{\omega} + \omega \times I_G \omega \end{bmatrix} = \begin{bmatrix} mg + \sum F_i \\ \sum (r_i - p_{com}) \times F_i \end{bmatrix}$$

This is simpler than full centroidal dynamics and sufficient for standing control.

---

## 🎯 Optimal Control Formulation

### Cost Function

The MPC minimizes a quadratic cost over the horizon $[t, t+T]$:

$$J = \int_{t}^{t+T} \left[ (x - x_{ref})^T Q (x - x_{ref}) + (u - u_{ref})^T R (u - u_{ref}) \right] dt$$

Where:
- $Q \in \mathbb{R}^{24 \times 24}$: State cost matrix
- $R \in \mathbb{R}^{24 \times 24}$: Input cost matrix
- $x_{ref}$: Reference state (standing pose)
- $u_{ref}$: Reference input (gravity compensation forces)

### Q Matrix (State Weights)

From `task.info`:

```
Q (scaled by 1.0):
┌─────────────────────────────────────────────────────┐
│ Momentum:     [15, 15, 30, 5, 10, 10]               │
│ Base pose:    [500, 500, 500]                       │
│ Base orient:  [100, 200, 200]                       │
│ Joints:       [20, 20, 20] × 4 legs                 │
└─────────────────────────────────────────────────────┘
```

**Interpretation:**
- High weight (500) on base position → Strong position tracking
- Medium weight (100-200) on orientation → Moderate orientation control
- Lower weight (15-30) on momentum → Allow momentum variation
- Weight 20 on joints → Soft joint tracking

### R Matrix (Input Weights)

```
R (scaled by 1e-3):
┌─────────────────────────────────────────────────────┐
│ Contact forces: [1, 1, 1] × 4 legs                  │
│ Foot velocities: [5000, 5000, 5000] × 4 legs        │
└─────────────────────────────────────────────────────┘
```

**Interpretation:**
- Low weight (1e-3) on forces → Forces are "cheap" to use
- High weight (5000 × 1e-3 = 5) on foot velocity → Penalize foot movement (standing!)

---

## 🔧 Whole Body Control (WBC)

### The Problem

MPC outputs **contact forces** $F_i$, but actuators need **joint torques** $\tau$. How to convert?

### Naive Approach: Jacobian Transpose

$$\tau = -J^T F$$

Where $J$ is the foot Jacobian. **Problem**: Ignores dynamics, gravity, Coriolis forces.

### Correct Approach: RNEA with External Forces

The **Recursive Newton-Euler Algorithm (RNEA)** computes inverse dynamics:

$$\tau = M(q)\ddot{q} + C(q, \dot{q})\dot{q} + G(q) - J^T F_{ext}$$

Where:
- $M(q)$: Mass matrix
- $C(q, \dot{q})$: Coriolis/centrifugal matrix
- $G(q)$: Gravity vector
- $F_{ext}$: External forces (contact forces from MPC)

### OCS2's WBC Implementation

`CentroidalModelRbdConversions::computeRbdTorqueFromCentroidalModelPD` uses:

$$\tau = \text{RNEA}(q_{des}, \dot{q}_{des}, \ddot{q}_{cmd}, F_{ext})$$

Where the commanded acceleration includes PD feedback:

$$\ddot{q}_{cmd} = \ddot{q}_{des} + K_p (q_{des} - q_{meas}) + K_d (\dot{q}_{des} - \dot{q}_{meas})$$

This ensures:
1. **Feedforward**: Desired trajectory tracking
2. **Feedback**: Error correction
3. **Contact forces**: Properly applied at end-effectors

---

## 📈 Gravity Compensation

### Standing Force Calculation

For standing with all 4 feet in contact:

$$F_z^{ref} = \frac{mg}{4} = \frac{52.1348 \times 9.81}{4} \approx 128 \text{ N per leg}$$

This is the **reference input** that the MPC uses:

```cpp
vector_t weightCompensatingInput(const CentroidalModelInfo& info,
                                 const contact_flag_t& contactFlags) {
    // Compute total vertical force needed
    scalar_t totalWeight = info.robotMass * 9.81;
    
    // Count active contacts
    int activeContacts = 0;
    for (bool flag : contactFlags) if (flag) activeContacts++;
    
    // Distribute force equally
    scalar_t forcePerLeg = totalWeight / activeContacts;
    
    // Set Fz for each leg in contact
    vector_t input = vector_t::Zero(info.inputDim);
    for (int i = 0; i < 4; i++) {
        if (contactFlags[i]) {
            input(3*i + 2) = forcePerLeg;  // Fz component
        }
    }
    return input;
}
```

---

## 🔄 Coordinate Transformations

### Quaternion to ZYX Euler

MuJoCo uses quaternions $(w, x, y, z)$, OCS2 uses ZYX Euler angles $(\psi, \theta, \phi)$:

```cpp
void quatToZYXEuler(double qw, double qx, double qy, double qz,
                   double& yaw, double& pitch, double& roll) {
    // Roll (X-axis rotation)
    double sinr_cosp = 2.0 * (qw * qx + qy * qz);
    double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
    roll = atan2(sinr_cosp, cosr_cosp);
    
    // Pitch (Y-axis rotation)
    double sinp = 2.0 * (qw * qy - qz * qx);
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp);  // Gimbal lock
    else
        pitch = asin(sinp);
    
    // Yaw (Z-axis rotation)
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    yaw = atan2(siny_cosp, cosy_cosp);
}
```

### Angular Velocity Transformation

OCS2's `getEulerAnglesZyxDerivativesFromGlobalAngularVelocity`:

$$\dot{\theta}_{ZYX} = T(\theta) \omega_{global}$$

Where $T$ is the transformation matrix from global angular velocity to Euler angle derivatives.

---

## 📊 Friction Cone Constraint

### Physical Meaning

Contact forces must lie within a **friction cone** to avoid slipping:

$$\sqrt{F_x^2 + F_y^2} \leq \mu F_z$$

Where $\mu = 0.5$ (friction coefficient from `task.info`).

### Pyramid Approximation

For computational efficiency, the cone is approximated by a pyramid with 4 faces:

$$|F_x| \leq \mu F_z, \quad |F_y| \leq \mu F_z$$

### Soft Constraint in OCS2

Using relaxed log barrier:

$$c_{friction} = \mu \cdot \log\left(1 + e^{(\sqrt{F_x^2 + F_y^2} - \mu F_z)/\delta}\right)$$

Parameters from `task.info`:
- `mu = 0.1`: Barrier weight
- `delta = 5.0`: Barrier smoothness

---

## 🧮 Summary of Key Formulas

| Formula | Description |
|---------|-------------|
| $\tau = \text{RNEA}(q, v, a, F_{ext})$ | Inverse dynamics |
| $h = A_G(q) \dot{q}$ | Centroidal momentum |
| $\dot{h} = mg + \sum F_i$ | Momentum dynamics |
| $F_z^{ref} = mg/4 \approx 128$ N | Gravity compensation |
| $\sqrt{F_x^2 + F_y^2} \leq \mu F_z$ | Friction cone |
| $J = \int (x-x_{ref})^T Q (x-x_{ref}) dt$ | MPC cost |

---

**Next:** [03_CODE_WALKTHROUGH.md](./03_CODE_WALKTHROUGH.md) - Line-by-Line Code Explanation
