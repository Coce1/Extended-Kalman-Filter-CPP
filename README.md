# Extended Kalman Filter (EKF) in C++

A modern C++ implementation of a non-linear state estimation pipeline utilizing an **Extended Kalman Filter (EKF)** with dynamic Jacobian calculation and real-time visualization.

---

## 📌 Project Overview

In real-world robotics and autonomous systems, sensors provide noisy measurements (e.g., GPS, IMU), and kinematic motions are inherently non-linear. This project implements an EKF to estimate the 2D pose and heading $[x, y, \theta]^T$ of a non-holonomic mobile agent.

### Key Features
- **Dynamic Jacobian Matrix:** Real-time computation of the transition Jacobian $F_k$ and of the measurement Jacobian $H_k$ (range to a known beacon).
- **Sensor Fusion Architecture:** Separated Prediction (odometry / kinematic model) and Update stages (linear GPS position + non-linear beacon range).
- **Matrix Operations:** Powered by Eigen fixed-size matrices (no heap allocation), LDLT solve instead of an explicit inverse, and Joseph-form covariance update.
- **Consistency Metrics:** Live RMSE (EKF vs raw GPS) and mean NIS (expected ~2 for GPS, ~1 for range) to check that the filter is well tuned.
- **Visual HUD:** Real-time graphical tracking displaying raw noisy measurements vs. smoothed EKF state estimations.

---

## 📐 Mathematical Formulation

### 1. State Vector & Kinematic Model
The state vector is defined as:

$$
x_k = \begin{bmatrix} x \\ y \\ \theta \end{bmatrix}_k
$$

The non-linear state transition function $f(x_{k-1}, u_k)$ with control inputs $u_k = [v, \omega]^T$:

$$
\begin{aligned}
x_k &= x_{k-1} + v \cdot \Delta t \cdot \cos(\theta_{k-1}) \\
y_k &= y_{k-1} + v \cdot \Delta t \cdot \sin(\theta_{k-1}) \\
\theta_k &= \theta_{k-1} + \omega \cdot \Delta t
\end{aligned}
$$

### 2. Jacobian Linearization
At each timestep, the state transition Jacobian $F_k$ is computed dynamically:

$$
F_k = \left. \frac{\partial f}{\partial x} \right|_{\hat{x}_{k-1}} =
\begin{bmatrix}
1 & 0 & -v \cdot \Delta t \cdot \sin(\theta) \\
0 & 1 & v \cdot \Delta t \cdot \cos(\theta) \\
0 & 0 & 1
\end{bmatrix}
$$

The process noise is derived from the odometry noise: $Q = G \, Q_u \, G^T$ with $Q_u = \mathrm{diag}(\sigma_v^2, \sigma_\omega^2)$.

### 3. Measurement Models
The GPS is linear, so $H = \begin{bmatrix} 1 & 0 & 0 \\ 0 & 1 & 0 \end{bmatrix}$. The range to a beacon at $(b_x, b_y)$ is non-linear, and its Jacobian is recomputed at each update:

$$
h(x) = \sqrt{(x-b_x)^2 + (y-b_y)^2}, \qquad H_k = \begin{bmatrix} \dfrac{x-b_x}{h} & \dfrac{y-b_y}{h} & 0 \end{bmatrix}
$$

---
## 🚀 Getting Started

### Prerequisites
- C++17 compatible compiler (MSVC, GCC, or Clang)
- CMake (3.15+) or Visual Studio 2026 (toolset `v145`)
- [Eigen 3](https://eigen.tuxfamily.org/) (Linear algebra library)
- [raylib](https://www.raylib.com/) (for real-time GUI & HUD rendering)

### Build & Run (Visual Studio)
1. Set the environment variable `EIGEN3_INCLUDE_DIR` to the folder that contains `Eigen/`, then restart Visual Studio.
2. Open `KalmanFilter.slnx` in **Visual Studio** and restore the NuGet packages (raylib).
3. Make sure the configuration is set to `Release` or `Debug` (**x64**).
4. Press **F5** (or click *Local Windows Debugger*) to compile and launch the simulation.

### Build & Run (CMake)
```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

### Controls & HUD
- **HUD display:** Displays real-time linear/angular velocity commands $(v, \omega)$, RMSE and mean NIS.
- **Red points:** Raw noisy GPS observations.
- **Blue circle:** Filtered, smooth trajectory estimated by the EKF.
- **Green:** Ground truth. **Purple:** Beacon.
- **R:** Reset the simulation.

## 📄 License

This project is licensed under the MIT License - see the LICENSE file for details.
