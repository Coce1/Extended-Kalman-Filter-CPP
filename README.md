# Extended Kalman Filter (EKF) in C++

A modern C++ implementation of a non-linear state estimation pipeline utilizing an **Extended Kalman Filter (EKF)** with dynamic Jacobian calculation and real-time visualization.

---

## 📌 Project Overview

In real-world robotics and autonomous systems, sensors provide noisy measurements (e.g., GPS, IMU), and kinematic motions are inherently non-linear. This project implements an EKF to estimate the 2D pose and heading $[x, y, \theta]^T$ of a non-holonomic mobile agent.

### Key Features
- **Dynamic Jacobian Matrix:** Real-time computation of transition Jacobians $F_k$ and measurement Jacobians $H_k$.
- **Sensor Fusion Architecture:** Separated Prediction (odometry / kinematic model) and Update (GPS / observation) stages.
- **Matrix Operations:** Powered by Eigen / Linear Algebra primitives for performance.
- **Visual HUD:** Real-time graphical tracking displaying raw noisy measurements vs. smoothed EKF state estimations.

---

## 📐 Mathematical Formulation

### 1. State Vector & Kinematic Model
The state vector is defined as:

$$
x_k = \begin{bmatrix} x \\\\ y \\\\ \theta \end{bmatrix}_k
$$

The non-linear state transition function $f(x_{k-1}, u_k)$ with control inputs $u_k = [v, \omega]^T$:

$$
\begin{aligned}
x_k &= x_{k-1} + v \cdot \Delta t \cdot \cos(\theta_{k-1}) \\\\
y_k &= y_{k-1} + v \cdot \Delta t \cdot \sin(\theta_{k-1}) \\\\
\theta_k &= \theta_{k-1} + \omega \cdot \Delta t
\end{aligned}
$$

### 2. Jacobian Linearization
At each timestep, the state transition Jacobian $F_k$ is computed dynamically:

$$
F_k = \left. \frac{\partial f}{\partial x} \right|_{\hat{x}_{k-1}} = \begin{bmatrix} 1 & 0 & -v \cdot \Delta t \cdot \sin(\theta) \\\\ 0 & 1 & v \cdot \Delta t \cdot \cos(\theta) \\\\ 0 & 0 & 1 \end{bmatrix}
$$

---
## 🛠 Project Structure

```text
.
├── main.cpp          # EKF loop, state updates, and visualization
├── .gitignore        # Ignores build artifacts and intermediate objects
└── README.md         # Project documentation and theoretical background

```
---
## 🚀 Getting Started

### Prerequisites
- C++17 compatible compiler (MSVC, GCC, or Clang)
- CMake (3.15+) or Visual Studio 2022
- [Eigen 3](https://eigen.tuxfamily.org/) (Linear algebra library)
- [raylib](https://www.raylib.com/) (for real-time GUI & HUD rendering)

### Build & Run (Visual Studio)
1. Open the project folder or `.sln` in **Visual Studio**.
2. Make sure the configuration is set to `Release` or `Debug` (**x64**).
3. Press **F5** (or click *Local Windows Debugger*) to compile and launch the simulation.

### Controls & HUD
- **HUD display:** Displays real-time linear/angular velocity commands $(v, \omega)$.
- **Red points:** Raw noisy GPS observations.
- **Blue circle:** Filtered, smooth trajectory estimated by the EKF.
