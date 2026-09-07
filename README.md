# AME 556 Final Project: 2D Biped Robot Control

Implementation of QP-based whole-body control for a 2D bipedal robot in MuJoCo, featuring physical constraint enforcement, standing control, and dynamic walking locomotion.

---

## Table of Contents

- [Code Architecture](#code-architecture)
- [Task 1: Physical Constraints and Simulation](#task-1-physical-constraints-and-simulation)
- [Task 2: Standing and Walking](#task-2-standing-and-walking)
- [Mathematical Formulation](#mathematical-formulation)
  - [Standing Controller (Inverse Dynamics QP)](#1-standing-controller-inverse-dynamics-qp)
  - [Walking Controller (Whole-Body QP)](#2-walking-controller-whole-body-qp)

---

## Code Architecture

The project follows a modular, layered architecture separating physics simulation, control computation, and visualization:

```
FP/
├── main.cpp              # Entry point and task scheduler
├── BipedRobot.h/cpp      # Robot state machine and locomotion interface
├── RobotController.h/cpp # QP-based control algorithms
├── SimulationUI.h/cpp    # Visualization, recording, and overlay
├── robot.xml             # MuJoCo robot model definition
└── task1_*/task2/        # Output data and plots
```

### Module Descriptions

| Module | Responsibility |
|--------|---------------|
| **`main.cpp`** | Initializes MuJoCo, orchestrates the task scheduler (`Task` class), manages simulation loop timing, and coordinates video/data recording |
| **`BipedRobot`** | Encapsulates robot state machine (IDLE, STAND, WALK phases), enforces physical constraints, manages Bezier foot trajectory generation, and interfaces with the low-level controller |
| **`RobotController`** | Implements two QP-based controllers: `computeStandControl()` for static balance and `computeWalkControl()` for dynamic locomotion with contact switching |
| **`SimulationUI`** | Provides GLFW window setup, mouse/keyboard callbacks, FFmpeg-based video recording (`VideoRecorder`), on-screen text overlay (`OverlayRenderer`), and CSV data logging (`DataLogger`) |
| **`robot.xml`** | Defines the 7-DOF planar biped: 3 floating base DOFs (x, z, pitch) + 4 joint DOFs (left/right hip/knee), with collision geometries and contact parameters |

### Control Flow Diagram

```
┌─────────────────────────────────────────────────────────────────────┐
│                           main.cpp                                  │
│  ┌─────────────┐    ┌─────────────┐    ┌──────────────────────────┐ │
│  │    Task     │───>│ BipedRobot  │───>│    RobotController       │ │
│  │  Scheduler  │    │State Machine│    │  (QP Optimization)       │ │
│  └─────────────┘    └─────────────┘    └──────────────────────────┘ │
│         │                  │                       │                │
│         v                  v                       v                │
│  ┌─────────────┐    ┌─────────────┐    ┌──────────────────────────┐ │
│  │ SimulationUI│<───│   MuJoCo    │<───│     Joint Torques        │ │
│  │  (Render)   │    │   Physics   │    │   τ = [τ₁,τ₂,τ₃,τ₄]ᵀ     │ │
│  └─────────────┘    └─────────────┘    └──────────────────────────┘ │
└─────────────────────────────────────────────────────────────────────┘
```

### State Machine Architecture

The `BipedRobot` class implements a finite state machine for locomotion:

```
                    ┌──────────┐
                    │   IDLE   │
                    └────┬─────┘
                         │ stand()
                         v
                    ┌──────────┐
                    │  STAND   │
                    └────┬─────┘
                         │ forwardWalk() / backwardWalk()
                         v
              ┌─────────────────────┐
              │    INIT_DS          │  (Initial Double Support)
              └──────────┬──────────┘
                         │
         ┌───────────────┴───────────────┐
         v                               v
┌─────────────────┐             ┌─────────────────┐
│  LEFT_SWING     │             │  RIGHT_SWING    │
│  (Right Stance) │             │  (Left Stance)  │
└────────┬────────┘             └────────┬────────┘
         │                               │
         v                               v
┌─────────────────┐             ┌─────────────────┐
│    DS_L2R       │────────────>│    DS_R2L       │
│ (Double Support)│<────────────│ (Double Support)│
└─────────────────┘             └─────────────────┘
```

---

## Task 1: Physical Constraints and Simulation

### Constraint Specifications

The simulation enforces the following physical constraints as specified in the project requirements:

| Joint Type | Constraint | Limit |
|------------|------------|-------|
| **Hip** $(q_1, q_3)$ | Angle | $-120° \leq q_{hip} \leq 30°$ |
| | Velocity | $\|\dot{q}_{hip}\| \leq 30 \text{ rad/s}$ |
| | Torque | $\|\tau_{hip}\| \leq 30 \text{ Nm}$ |
| **Knee** $(q_2, q_4)$ | Angle | $0° \leq q_{knee} \leq 160°$ |
| | Velocity | $\|\dot{q}_{knee}\| \leq 15 \text{ rad/s}$ |
| | Torque | $\|\tau_{knee}\| \leq 60 \text{ Nm}$ |

### Implementation Details

**Constraint Checking** (`BipedRobot::checkConstraints()`):
- Evaluates all joint angles and velocities at each simulation step
- Sets violation flag and generates descriptive warning message
- Terminates simulation upon any constraint violation

**Torque Saturation** (`BipedRobot::enforceTorqueLimits()`):
- Applies input saturation before sending torques to actuators:

$$
\tau_i = \begin{cases}
\tau_{max} & \text{if } \tau_i \geq \tau_{max} \\\\
-\tau_{max} & \text{if } \tau_i \leq -\tau_{max} \\\\
\tau_i & \text{otherwise}
\end{cases}
$$

### Demonstration Cases

#### Case 1.1: Joint Angle Limit Violation (Hip)

Initial configuration designed to cause hip angle violation during free fall:

<p align="center">
  <img src="task1_1/task1_1.gif" alt="Task 1.1 - Hip Angle Violation" width="66%">
</p>
<p align="center">
  <em>Hip joint angle limit violation demonstration</em>
</p>

<p align="center">
  <img src="task1_1/fig1_trunk_x.png" alt="Trunk X" width="48%">
  <img src="task1_1/fig2_trunk_y.png" alt="Trunk Y" width="48%">
</p>
<p align="center">
  <em>Trunk position (X and Y)</em>
</p>

<p align="center">
  <img src="task1_1/fig3_trunk_pitch.png" alt="Trunk Pitch" width="48%">
  <img src="task1_1/fig8_joint_torques.png" alt="Joint Torques" width="48%">
</p>
<p align="center">
  <em>Trunk pitch and joint torques</em>
</p>

<p align="center">
  <img src="task1_1/fig4_left_hip.png" alt="Left Hip Angle" width="48%">
  <img src="task1_1/fig5_left_knee.png" alt="Left Knee Angle" width="48%">
</p>
<p align="center">
  <em>Left leg joint states</em>
</p>

<p align="center">
  <img src="task1_1/fig6_right_hip.png" alt="Right Hip Angle" width="48%">
  <img src="task1_1/fig7_right_knee.png" alt="Right Knee Angle" width="48%">
</p>
<p align="center">
  <em>Right leg joint states</em>
</p>

#### Case 1.2: Joint Angle Limit Violation (Knee)

Configuration causing knee angle constraint violation:

<p align="center">
  <img src="task1_2/task1_2.gif" alt="Task 1.2 - Knee Angle Violation" width="66%">
</p>
<p align="center">
  <em>Knee joint angle limit violation demonstration</em>
</p>

<p align="center">
  <img src="task1_2/fig1_trunk_x.png" alt="Trunk X" width="48%">
  <img src="task1_2/fig2_trunk_y.png" alt="Trunk Y" width="48%">
</p>
<p align="center">
  <em>Trunk position (X and Y)</em>
</p>

<p align="center">
  <img src="task1_2/fig3_trunk_pitch.png" alt="Trunk Pitch" width="48%">
  <img src="task1_2/fig8_joint_torques.png" alt="Joint Torques" width="48%">
</p>
<p align="center">
  <em>Trunk pitch and joint torques</em>
</p>

<p align="center">
  <img src="task1_2/fig4_left_hip.png" alt="Left Hip Angle" width="48%">
  <img src="task1_2/fig5_left_knee.png" alt="Left Knee Angle" width="48%">
</p>
<p align="center">
  <em>Left leg joint states</em>
</p>

<p align="center">
  <img src="task1_2/fig6_right_hip.png" alt="Right Hip Angle" width="48%">
  <img src="task1_2/fig7_right_knee.png" alt="Right Knee Angle" width="48%">
</p>
<p align="center">
  <em>Right leg joint states</em>
</p>

#### Case 1.3: Joint Velocity Limit Violation

High-energy initial state causing velocity constraint violation:

<p align="center">
  <img src="task1_3/task1_3.gif" alt="Task 1.3 - Velocity Violation" width="66%">
</p>
<p align="center">
  <em>Joint velocity limit violation demonstration</em>
</p>

<p align="center">
  <img src="task1_3/fig1_trunk_x.png" alt="Trunk X" width="48%">
  <img src="task1_3/fig2_trunk_y.png" alt="Trunk Y" width="48%">
</p>
<p align="center">
  <em>Trunk position (X and Y)</em>
</p>

<p align="center">
  <img src="task1_3/fig3_trunk_pitch.png" alt="Trunk Pitch" width="48%">
  <img src="task1_3/fig8_joint_torques.png" alt="Joint Torques" width="48%">
</p>
<p align="center">
  <em>Trunk pitch and joint torques</em>
</p>

<p align="center">
  <img src="task1_3/fig4_left_hip.png" alt="Left Hip States" width="48%">
  <img src="task1_3/fig5_left_knee.png" alt="Left Knee States" width="48%">
</p>
<p align="center">
  <em>Left leg joint states</em>
</p>

<p align="center">
  <img src="task1_3/fig6_right_hip.png" alt="Right Hip States" width="48%">
  <img src="task1_3/fig7_right_knee.png" alt="Right Knee States" width="48%">
</p>
<p align="center">
  <em>Right leg joint states</em>
</p>

---

## Task 2: Standing and Walking

### Task Sequence

The complete demonstration follows this timeline:

| Time (s) | Phase | Description |
|----------|-------|-------------|
| 0.0 - 1.0 | Warmup | Initialize standing at $y_d = 0.45$ m |
| 1.0 - 1.5 | Stand Up | $y_d: 0.45 \rightarrow 0.55$ m (0.5s) |
| 1.5 - 2.5 | Stand Down | $y_d: 0.55 \rightarrow 0.40$ m (1.0s) |
| 2.5 - 6.0 | Prepare | Stabilize at $y_d = 0.48$ m |
| 6.0 - 12.44 | Forward Walk | Continuous forward locomotion |
| 12.44 - 14.0 | Transition | Standing stabilization |
| 14.0 - 19.5 | Backward Walk | Continuous backward locomotion |
| 19.5+ | Final Stand | Terminal standing pose |

### Walking Results

<p align="center">
  <img src="task2/task2.gif" alt="Task 2 - Complete Walking Demo" width="80%">
</p>
<p align="center">
  <em>Complete standing and walking demonstration</em>
</p>

#### Trunk Trajectory Tracking

<p align="center">
  <img src="task2/fig1_trunk_x.png" alt="Trunk X Position" width="48%">
  <img src="task2/fig2_trunk_y.png" alt="Trunk Y Position" width="48%">
</p>
<p align="center">
  <em>Trunk position tracking (X: forward displacement, Y: height regulation)</em>
</p>

<p align="center">
  <img src="task2/fig3_trunk_pitch.png" alt="Trunk Pitch" width="66%">
</p>
<p align="center">
  <em>Trunk pitch angle regulation during locomotion</em>
</p>

#### Joint Angle Trajectories

<p align="center">
  <img src="task2/fig4_left_hip.png" alt="Left Hip" width="48%">
  <img src="task2/fig5_left_knee.png" alt="Left Knee" width="48%">
</p>
<p align="center">
  <em>Left leg joint angles (all within physical limits)</em>
</p>

<p align="center">
  <img src="task2/fig6_right_hip.png" alt="Right Hip" width="48%">
  <img src="task2/fig7_right_knee.png" alt="Right Knee" width="48%">
</p>
<p align="center">
  <em>Right leg joint angles (all within physical limits)</em>
</p>

#### Control Torques

<p align="center">
  <img src="task2/fig8_joint_torques.png" alt="Joint Torques" width="80%">
</p>
<p align="center">
  <em>Joint torques throughout the demonstration (all within saturation limits)</em>
</p>

---

## Mathematical Formulation

### Robot Model

The 2D bipedal robot has 7 degrees of freedom:

$$
\mathbf{q} = \left[\begin{array}{c} x \\\\ z \\\\ \theta \\\\ q_1 \\\\ q_2 \\\\ q_3 \\\\ q_4 \end{array}\right] = \left[\begin{array}{c} \text{trunk } x \\\\ \text{trunk } z \\\\ \text{trunk pitch} \\\\ \text{left hip} \\\\ \text{left knee} \\\\ \text{right hip} \\\\ \text{right knee} \end{array}\right] \in \mathbb{R}^7
$$

The equations of motion follow the standard manipulator form:

$$
\mathbf{M}(\mathbf{q})\ddot{\mathbf{q}} + \mathbf{h}(\mathbf{q}, \dot{\mathbf{q}}) = \mathbf{S}^T \vec{\tau} + \mathbf{J}_c^T \vec{\lambda}
$$

where:
- $\mathbf{M} \in \mathbb{R}^{7 \times 7}$: Mass matrix
- $\mathbf{h} \in \mathbb{R}^{7}$: Coriolis, centrifugal, and gravity forces
- $\mathbf{S} = [\mathbf{0}_{4\times3} \mid \mathbf{I}_4] \in \mathbb{R}^{4 \times 7}$: Selection matrix for actuated joints
- $\vec{\tau} \in \mathbb{R}^4$: Joint torques
- $\mathbf{J}_c \in \mathbb{R}^{n_c \times 7}$: Contact Jacobian
- $\vec{\lambda} \in \mathbb{R}^{n_c}$: Contact forces

### 1. Standing Controller (Inverse Dynamics QP)

For static double-support standing, the controller computes desired trunk accelerations via PD control, then solves for joint torques through inverse dynamics with QP optimization.

#### Step 1: Compute Desired Base Acceleration

$$
\ddot{\mathbf{q}}_{base}^{des} = \left[\begin{array}{c} \ddot{x}^{des} \\\\ \ddot{z}^{des} \\\\ \ddot{\theta}^{des} \end{array}\right] = \left[\begin{array}{c} K_p^x(x_d - x) - K_d^x \dot{x} \\\\ K_p^z(z_d - z) - K_d^z \dot{z} \\\\ K_p^\theta(\theta_d - \theta) - K_d^\theta \dot{\theta} \end{array}\right]
$$

**Gains:** $K_p^x = 100$, $K_d^x = 20$, $K_p^z = 200$, $K_d^z = 20$, $K_p^\theta = 200$, $K_d^\theta = 20$

#### Step 2: Compute Desired Joint Acceleration via Foot Constraint

With both feet on the ground, foot velocities and accelerations must be zero:

$$
\mathbf{J}_{task}\ddot{\mathbf{q}} + \dot{\mathbf{J}}_{task}\dot{\mathbf{q}} = \mathbf{0}
$$

The task Jacobian stacks the X and Z components of both feet:

$$
\mathbf{J}_{task} = \left[\begin{array}{c} J_{L,x} \\\\ J_{L,z} \\\\ J_{R,x} \\\\ J_{R,z} \end{array}\right] \in \mathbb{R}^{4 \times 7}
$$

Partitioning into base and joint components:

$$
\mathbf{J}_{task} = \begin{bmatrix} \mathbf{J}_{base} & \mathbf{J}_{joint} \end{bmatrix}
$$

where:

$$\mathbf{J}_{\text{base}} \in \mathbb{R}^{4 \times 3}, \quad \mathbf{J}_{\text{joint}} \in \mathbb{R}^{4 \times 4}$$

The desired joint acceleration is:

$$
\ddot{\mathbf{q}}_{joint}^{des} = -\mathbf{J}_{joint}^{-1}\mathbf{J}_{base}\ddot{\mathbf{q}}_{base}^{des}
$$

#### Step 3: Inverse Dynamics

The total desired acceleration:

$$
\ddot{\mathbf{q}}^{des} = \left[\begin{array}{c} \ddot{\mathbf{q}}_{base}^{des} \\\\ \ddot{\mathbf{q}}_{joint}^{des} \end{array}\right]
$$

Compute inverse dynamics target:

$$
\vec{\tau}_{ID} = \mathbf{M}\ddot{\mathbf{q}}^{des} + \mathbf{h}
$$

#### Step 4: QP Formulation

**Decision Variables:**

$$
\mathbf{x} = \left[\begin{array}{c} \vec{\tau} \\\\ \vec{\lambda} \end{array}\right] = \left[\begin{array}{c} \tau_1 \\\\ \tau_2 \\\\ \tau_3 \\\\ \tau_4 \\\\ f_{L,x} \\\\ f_{L,z} \\\\ f_{R,x} \\\\ f_{R,z} \end{array}\right] \in \mathbb{R}^{8}
$$

**Cost Function:**

$$
\min_{\mathbf{x}} \frac{1}{2}\mathbf{x}^T\mathbf{P}\mathbf{x}
$$

$$
\mathbf{P} = \left[\begin{array}{cc} \mathbf{I}_4 & \mathbf{0} \\\\ \mathbf{0} & 0.1 \cdot \mathbf{I}_4 \end{array}\right] \in \mathbb{R}^{8 \times 8}
$$

**Equality Constraints (Dynamics):**

$$
\mathbf{S}^T \vec{\tau} + \mathbf{J}_c^T \vec{\lambda} = \vec{\tau}_{ID}
$$

Expanded form:

$$
\underbrace{\left[\begin{array}{cccccccc}
0 & 0 & 0 & 0 & J_{L,x}^{(1)} & J_{L,z}^{(1)} & J_{R,x}^{(1)} & J_{R,z}^{(1)} \\\\
0 & 0 & 0 & 0 & J_{L,x}^{(2)} & J_{L,z}^{(2)} & J_{R,x}^{(2)} & J_{R,z}^{(2)} \\\\
0 & 0 & 0 & 0 & J_{L,x}^{(3)} & J_{L,z}^{(3)} & J_{R,x}^{(3)} & J_{R,z}^{(3)} \\\\
1 & 0 & 0 & 0 & J_{L,x}^{(4)} & J_{L,z}^{(4)} & J_{R,x}^{(4)} & J_{R,z}^{(4)} \\\\
0 & 1 & 0 & 0 & J_{L,x}^{(5)} & J_{L,z}^{(5)} & J_{R,x}^{(5)} & J_{R,z}^{(5)} \\\\
0 & 0 & 1 & 0 & J_{L,x}^{(6)} & J_{L,z}^{(6)} & J_{R,x}^{(6)} & J_{R,z}^{(6)} \\\\
0 & 0 & 0 & 1 & J_{L,x}^{(7)} & J_{L,z}^{(7)} & J_{R,x}^{(7)} & J_{R,z}^{(7)}
\end{array}\right]}_{\mathbf{A}_{eq} \in \mathbb{R}^{7 \times 8}}
\left[\begin{array}{c} \tau_1 \\\\ \tau_2 \\\\ \tau_3 \\\\ \tau_4 \\\\ f_{L,x} \\\\ f_{L,z} \\\\ f_{R,x} \\\\ f_{R,z} \end{array}\right] = \vec{\tau}_{ID}
$$

where $J_{L,x}^{(i)}$ denotes the $i$-th row element of the left foot X-direction Jacobian.

**Inequality Constraints:**

*Friction Cone (linearized):*

$$
|f_x| \leq \mu f_z \quad \Rightarrow \quad -\mu f_z \leq f_x \leq \mu f_z
$$

For each foot:

$$f_{L,x} - \mu f_{L,z} \leq 0$$

$$-f_{L,x} - \mu f_{L,z} \leq 0$$

$$f_{R,x} - \mu f_{R,z} \leq 0$$

$$-f_{R,x} - \mu f_{R,z} \leq 0$$

*Normal Force Bounds:*

$$
0 \leq f_{L,z} \leq 500, \quad 0 \leq f_{R,z} \leq 500
$$

*Torque Limits:*

$$
|\tau_1|, |\tau_3| \leq 30 \text{ Nm}, \quad |\tau_2|, |\tau_4| \leq 60 \text{ Nm}
$$

---

### 2. Walking Controller (Whole-Body QP)

The walking controller handles dynamic locomotion with contact switching, formulated as a task-space whole-body QP.

#### Decision Variables

$$
\mathbf{x} = \left[\begin{array}{c} \ddot{\mathbf{q}} \\\\ \vec{\tau} \\\\ \vec{\lambda} \end{array}\right] \in \mathbb{R}^{15}
$$

Explicitly:

$$
\mathbf{x} = \left[\begin{array}{c} \ddot{x} \\\\ \ddot{z} \\\\ \ddot{\theta} \\\\ \ddot{q}_1 \\\\ \ddot{q}_2 \\\\ \ddot{q}_3 \\\\ \ddot{q}_4 \\\\ \tau_1 \\\\ \tau_2 \\\\ \tau_3 \\\\ \tau_4 \\\\ f_{L,x} \\\\ f_{L,z} \\\\ f_{R,x} \\\\ f_{R,z} \end{array}\right]
$$

#### Cost Function (Task-Space Tracking)

$$
\min_{\mathbf{x}} \sum_{k} w_k \|\mathbf{J}_k \ddot{\mathbf{q}} - \ddot{\mathbf{p}}_k^{des}\|^2 + \epsilon_q \|\ddot{\mathbf{q}}\|^2 + \epsilon_\tau \|\vec{\tau}\|^2 + \epsilon_\lambda \|\vec{\lambda}\|^2
$$

**Task Definitions:**

| Task | Weight | Jacobian | Desired Acceleration |
|------|--------|----------|---------------------|
| Trunk X | $w_x$ | $\mathbf{J}_x = [1, 0, 0, 0, 0, 0, 0]$ | $\ddot{x}^{des} = K_p^x(x_d - x) + K_d^x(\dot{x}_d - \dot{x})$ |
| Trunk Z | $w_z$ | $\mathbf{J}_z = [0, 1, 0, 0, 0, 0, 0]$ | $\ddot{z}^{des} = K_p^z(z_d - z) - K_d^z \dot{z}$ |
| Trunk Pitch | $w_\theta$ | $\mathbf{J}_\theta = [0, 0, 1, 0, 0, 0, 0]$ | $\ddot{\theta}^{des} = K_p^\theta(\theta_d - \theta) - K_d^\theta \dot{\theta}$ |
| Swing Foot | $w_{\text{sw}}$ | $\mathbf{J}_{\text{sw}} \in \mathbb{R}^{2 \times 7}$ | $\ddot{\mathbf{p}}_{\text{sw}}^{\text{des}}$ (from Bezier trajectory) |

**Walking Gains:** $K_p^x = 100$, $K_d^x = 15$, $K_p^z = 300$, $K_d^z = 30$, $K_p^\theta = 300$, $K_d^\theta = 30$, $K_p^{\text{sw}} = 450$, $K_d^{\text{sw}} = 20$

The quadratic cost matrix structure:

$$
\mathbf{P} = \sum_k w_k \mathbf{J}_k^T \mathbf{J}_k + \left[\begin{array}{ccc} \epsilon_q \mathbf{I}_7 & \mathbf{0} & \mathbf{0} \\\\ \mathbf{0} & \epsilon_\tau \mathbf{I}_4 & \mathbf{0} \\\\ \mathbf{0} & \mathbf{0} & \epsilon_\lambda \mathbf{I}_4 \end{array}\right]
$$

with $\epsilon_q = 0.01$, $\epsilon_\tau = 10^{-4}$, $\epsilon_\lambda = 10^{-4}$.

#### Equality Constraints

**1. Equations of Motion:**

$$
\mathbf{M}\ddot{\mathbf{q}} - \mathbf{S}^T \vec{\tau} - \mathbf{J}_c^T \vec{\lambda} = -\mathbf{h}
$$

In matrix form (rows for each DOF):

$$
\underbrace{\left[\begin{array}{ccccccccccc}
M_{11} & \cdots & M_{17} & 0 & 0 & 0 & 0 & -J_{L,x}^{(1)} & -J_{L,z}^{(1)} & -J_{R,x}^{(1)} & -J_{R,z}^{(1)} \\\\
\vdots & \ddots & \vdots & \vdots & \vdots & \vdots & \vdots & \vdots & \vdots & \vdots & \vdots \\\\
M_{71} & \cdots & M_{77} & 0 & 0 & 0 & -1 & -J_{L,x}^{(7)} & -J_{L,z}^{(7)} & -J_{R,x}^{(7)} & -J_{R,z}^{(7)}
\end{array}\right]}_{\mathbf{A}_{dyn} \in \mathbb{R}^{7 \times 15}}
\mathbf{x} = -\mathbf{h}
$$

The selection matrix structure embeds as:

$$
\mathbf{S}^T = \left[\begin{array}{c} \mathbf{0}_{3 \times 4} \\\\ \mathbf{I}_4 \end{array}\right]
$$

**2. Contact Constraints (for stance foot):**

If foot $k$ is in contact:

$$
\mathbf{J}_k \ddot{\mathbf{q}} = \mathbf{0} \quad \text{(zero foot acceleration)}
$$

**3. Zero Force on Swing Foot:**

If foot $k$ is swinging:

$$
\vec{\lambda}_k = \mathbf{0}
$$

#### Inequality Constraints

**Friction Cone (for stance feet only):**

$$
\left[\begin{array}{cc} 1 & -\mu \\\\ -1 & -\mu \end{array}\right] \left[\begin{array}{c} f_x \\\\ f_z \end{array}\right] \leq \mathbf{0}
$$

**Normal Force Bounds:**

$$
0 \leq f_z \leq 500 \text{ N}
$$

**Torque Limits:**

$$
-30 \leq \tau_{hip} \leq 30, \quad -60 \leq \tau_{knee} \leq 60
$$

#### Complete Constraint Matrix Structure

The full constraint matrix $\mathbf{A} \in \mathbb{R}^{n_c \times 15}$ (where $n_c$ varies with contact state):

$$
\mathbf{A} = \left[\begin{array}{c}
\mathbf{A}_{dyn} \\\\
\mathbf{A}_{contact} \\\\
\mathbf{A}_{force} \\\\
\mathbf{A}_{friction} \\\\
\mathbf{A}_{torque}
\end{array}\right]
$$

For **double support** (both feet in contact):

$$
\mathbf{A}_{contact} = \left[\begin{array}{ccccccccccc}
J_{L,x}^{(1)} & \cdots & J_{L,x}^{(7)} & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\\\
J_{L,z}^{(1)} & \cdots & J_{L,z}^{(7)} & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\\\
J_{R,x}^{(1)} & \cdots & J_{R,x}^{(7)} & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\\\
J_{R,z}^{(1)} & \cdots & J_{R,z}^{(7)} & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0
\end{array}\right]
$$

For **single support** (e.g., left swing, right stance):

$$
\mathbf{A}_{contact} = \left[\begin{array}{ccccccccccc}
J_{R,x}^{(1)} & \cdots & J_{R,x}^{(7)} & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\\\
J_{R,z}^{(1)} & \cdots & J_{R,z}^{(7)} & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0
\end{array}\right]
$$

$$
\mathbf{A}_{force}^{swing} = \left[\begin{array}{ccccccccccc}
0 & \cdots & 0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 & 0 \\\\
0 & \cdots & 0 & 0 & 0 & 0 & 0 & 0 & 1 & 0 & 0
\end{array}\right]
$$

#### Bezier Foot Trajectory

The swing foot follows a cubic Bezier curve with 4 control points:

$$
\mathbf{p}(s) = (1-s)^3\mathbf{P}_0 + 3(1-s)^2 s\mathbf{P}_1 + 3(1-s)s^2\mathbf{P}_2 + s^3\mathbf{P}_3
$$

where $s = t/T_{swing} \in [0,1]$ and:

$$\mathbf{P}_0 = \text{Initial foot position}$$

$$\mathbf{P}_1 = \mathbf{P}_0 + \left[\begin{array}{c} v_{\text{trunk}} \cdot T/3 \\\\ 0 \\\\ h_{\text{clearance}} \end{array}\right]$$

$$\mathbf{P}_2 = \mathbf{P}_3 + \left[\begin{array}{c} -v_{\text{trunk}} \cdot T/6 \\\\ 0 \\\\ 0.1 \cdot h_{\text{clearance}} \end{array}\right]$$

$$\mathbf{P}_3 = \text{Target landing position}$$

Velocity and acceleration are computed analytically:

$$
\dot{\mathbf{p}}(s) = \frac{1}{T}\frac{d\mathbf{p}}{ds} = \frac{1}{T}\left[3(1-s)^2(\mathbf{P}_1-\mathbf{P}_0) + 6(1-s)s(\mathbf{P}_2-\mathbf{P}_1) + 3s^2(\mathbf{P}_3-\mathbf{P}_2)\right]
$$

$$
\ddot{\mathbf{p}}(s) = \frac{1}{T^2}\frac{d^2\mathbf{p}}{ds^2} = \frac{1}{T^2}\left[6(1-s)(\mathbf{P}_2-2\mathbf{P}_1+\mathbf{P}_0) + 6s(\mathbf{P}_3-2\mathbf{P}_2+\mathbf{P}_1)\right]
$$

---
