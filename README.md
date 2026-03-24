# Denso VS050 — Dual Quaternion Kinematics & Constrained Motion Control

**RAS 545 · Bonus Assignment · Arizona State University · Fall 2025**

> DQ Robotics-based simulation of the Denso VS050 6-DOF manipulator: dual quaternion forward kinematics, R⊥R‖R FK verification against homogeneous transforms, and QP-based constrained motion control with plane and sphere constraints.

---

## Overview

This project implements advanced robot kinematics and control using the **DQ Robotics MATLAB toolbox** for the **Denso VS050** 6-DOF industrial robot. It covers 8 DQ Robotics lessons and culminates in constrained trajectory control where the end-effector must navigate around geometric obstacles defined as plane and sphere constraints, solved via Quadratic Programming (QP).

---

## Repository Structure
```
.
├── lesson_1_sol.mlx                    # DQ fundamentals
├── lesson_2_sol.mlx                    # DQ algebra and operations
├── lesson_3_sol.mlx                    # Rigid body transformations
├── lesson_4_sol.mlx                    # Forward kinematics with DQ
├── lesson_5_sol.mlx                    # Jacobians in DQ framework
├── lesson_6_sol.mlx                    # DQ-based control
├── lesson_7_sol.mlx                    # Advanced DQ control
├── Forward_Kinematics_R_RR.mlx         # R⊥R‖R FK: DQ vs homogeneous transform
├── lesson_8/
│   ├── VS050RobotDH.m                  # Denso VS050 DH parameters (DQ Robotics)
│   ├── vs050_plane_constraints.m       # QP control with 6-plane cube constraint
│   ├── vs050_entry_sphere_constraint.m # QP control with sphere entry constraint
│   └── DQ_QPOASESSolver.m              # qpOASES wrapper for DQ controller
├── Bonus_Homogeneous_transf.pdf        # Handwritten homogeneous transform derivation
├── lesson_2_homework.pdf               # Lesson 2 written homework
├── lesson_7_homework.pdf               # Lesson 7 written homework
└── README.md
```

---

## Denso VS050 DH Parameters

Full 6-DOF kinematic model implemented using standard DH convention:

| Joint | θ (rad) | d (m) | a (m) | α (rad) | Type |
|---|---|---|---|---|---|
| 1 | 0 | 0.345 | 0.075 | π/2 | Revolute |
| 2 | 0 | 0 | 0.365 | 0 | Revolute |
| 3 | 0 | 0 | 0.035 | π/2 | Revolute |
| 4 | 0 | 0.305 | 0 | −π/2 | Revolute |
| 5 | 0 | 0 | 0 | π/2 | Revolute |
| 6 | 0 | 0.075 | 0 | 0 | Revolute |

End-effector offset: **+20 cm along z-axis** via DQ effector transform.

---

## Key Components

### Lesson 8 — Constrained Motion Control

The VS050 must reach target positions while satisfying geometric constraints enforced as QP inequality constraints on the task-space Jacobian.

**Plane constraints** (`vs050_plane_constraints.m`):
- Defines a 5 cm cube centered at `[0.3, 0.1, 0.5]` m
- 6 half-space constraints keep the end-effector outside the cube
- Controller: `DQ_ClassicQPController` with translation objective

**Sphere constraint** (`vs050_entry_sphere_constraint.m`):
- Constrains end-effector entry direction into a spherical workspace region
- Uses DQ translation Jacobian for real-time constraint evaluation

**Controller parameters:**

| Parameter | Value |
|---|---|
| Gain λ | 3 |
| Damping η | 0.1 |
| Stability threshold | 0.001 |
| Time step dt | 0.01 s |
| Solver | qpOASES via `DQ_QPOASESSolver` |

### R⊥R‖R Forward Kinematics

`Forward_Kinematics_R_RR.mlx` derives FK for a 3-DOF R⊥R‖R robot using both:
- **Dual quaternion** method (DQ Robotics)
- **Homogeneous transformation matrices** (classical method)

Results compared for position and orientation to verify equivalence.

---

## Setup & Usage

### Requirements
```matlab
% Install DQ Robotics toolbox
% https://dqrobotics.github.io/

% Install qpOASES (required for Lesson 8 constrained control)
% https://github.com/coin-or/qpOASES
```

### Run Lesson 8 — Plane Constraints
```matlab
addpath('lesson_8/');
vs050_plane_constraints
```

### Run FK Comparison
```matlab
Forward_Kinematics_R_RR   % opens as MATLAB Live Script
```

---

## Lessons Learned

- Dual quaternions represent both rotation and translation in a single compact algebraic structure — more numerically stable than homogeneous matrices for chained transforms
- QP-based constrained control allows real-time geometric obstacle avoidance without trajectory pre-planning
- The VS050's DH parameters require careful unit handling — d and a values in meters, α in radians
- qpOASES must be compiled separately from DQ Robotics — missing this causes runtime errors in the constrained controller

---

## Course Info

- **Course:** RAS 545 — Robotic and Autonomous Systems
- **Instructor:** Prof. Mostafa Yourdkhani
- **University:** Arizona State University, Tempe AZ
- **Semester:** Fall 2025
- **Assignment:** Bonus (30 pts)
