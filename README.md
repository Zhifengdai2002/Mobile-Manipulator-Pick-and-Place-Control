# Mobile Manipulator Pick-and-Place Control

A modular Python implementation of a mobile manipulator pick-and-place system using **SE(3) trajectory tracking**, **PI feedback control**, and **Jacobian pseudoinverse inverse kinematics**.

The project plans an 8-stage end-effector trajectory for grasping, transporting, and placing a cube, then tracks that trajectory in closed loop with a wheeled mobile base and a 5-DOF arm.

---

## Overview

This project simulates a mobile manipulator performing an autonomous pick-and-place task:

1. Move above the cube  
2. Descend to grasp pose  
3. Close gripper  
4. Lift the cube  
5. Move to the goal location  
6. Descend to place pose  
7. Open gripper  
8. Retreat to standoff pose  

The controller computes the end-effector tracking error in **SE(3)** using the matrix logarithm, then maps the desired 6D end-effector twist to **4 wheel velocities + 5 joint velocities** through the full robot Jacobian pseudoinverse.

---

## Features

- **Task-space trajectory generation** in SE(3)
- **8-segment pick-and-place pipeline**
- **PI feedback controller** on SE(3)
- **Velocity-level inverse kinematics**
- **Jacobian pseudoinverse** for redundant actuation
- **Mobile base + manipulator coupling**
- **Trajectory tracking under non-ideal initial conditions**
- Exportable robot configuration and tracking error data for visualization and analysis

---

## Robot Model

This system represents a mobile manipulator with:

- **3 chassis configuration variables**: orientation, x, y
- **5 arm joints**
- **4 wheel actuators**
- **1 gripper state**

Important distinctions:

- **Mechanical DOF:** 8  
  - 3 from the mobile base  
  - 5 from the arm
- **Control input dimension:** 9  
  - 4 wheel velocities  
  - 5 joint velocities
- **Task-space output:** 6  
  - 3 angular velocity components  
  - 3 linear velocity components

This makes the controller a **redundant 9-to-6 inverse kinematics problem**.

---
