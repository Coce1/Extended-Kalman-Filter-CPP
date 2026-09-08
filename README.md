# 🛰️ Extended Kalman Filter (EKF) in C++

A robust, real-time **Extended Kalman Filter (EKF)** implementation in modern C++ for non-linear state estimation and sensor fusion, visualized using **Raylib** and powered by **Eigen 3**.

---

## 📌 Overview

The Extended Kalman Filter (EKF) is the standard algorithm for state estimation in non-linear dynamic systems. This project models a mobile robot tracking problem where non-linear kinematic motions and noisy sensor measurements (GPS/Position and Odometry/IMU) are fused to reconstruct the true state trajectory with minimal covariance error.

---

## ✨ Features

- **Non-Linear State Estimation**: Full EKF formulation linearizing dynamics and observation models via Jacobian matrices.
- **Sensor Fusion**: Integrates noisy multi-rate measurements (simulated GPS position updates and kinematic velocity readings).
- **Real-Time Visualization**: 2D hardware-accelerated interactive display using **Raylib** to show Ground Truth, Raw Measurements, and Estimated States with uncertainty ellipses.
- **High Performance**: Optimized matrix computations powered by the **Eigen 3** linear algebra library.

---

## 📐 Mathematical Formulation

### 1. State Vector
The discrete system state $\mathbf{x}_k \in \mathbb{R}^4$ is defined by 2D coordinates and linear velocities:

```math
\mathbf{x}_k = \begin{bmatrix} x_k \\ y_k \\ \dot{x}_k \\ \dot{y}_k \end{bmatrix}

### 2. Prediction Step
Using a non-linear process model $f(\mathbf{x}_{k-1}, \mathbf{u}_k)$ with sampling time $\Delta t$:

$$\mathbf{\hat{x}}_k^- = f(\mathbf{\hat{x}}_{k-1}^+, \mathbf{u}_k)$$

$$\mathbf{P}_k^- = \mathbf{F}_k \mathbf{P}_{k-1}^+ \mathbf{F}_k^T + \mathbf{Q}$$

Where:
- $\mathbf{F}_k = \left. \frac{\partial f}{\partial \mathbf{x}} \right\vert{}_{\mathbf{\hat{x}}_{k-1}^+}$ is the process Jacobian.
- $\mathbf{Q}$ is the process noise covariance matrix.
- $\mathbf{P}$ is the state estimation error covariance matrix.

### 3. Update Step
Given a measurement vector $\mathbf{z}_k$ and observation model $h(\mathbf{x})$:

$$\mathbf{y}_k = \mathbf{z}_k - h(\mathbf{\hat{x}}_k^-) \quad \text{(Innovation)}$$

$$\mathbf{S}_k = \mathbf{H}_k \mathbf{P}_k^- \mathbf{H}_k^T + \mathbf{R} \quad \text{(Innovation Covariance)}$$

$$\mathbf{K}_k = \mathbf{P}_k^- \mathbf{H}_k^T \mathbf{S}_k^{-1} \quad \text{(Kalman Gain)}$$

$$\mathbf{\hat{x}}_k^+ = \mathbf{\hat{x}}_k^- + \mathbf{K}_k \mathbf{y}_k \quad \text{(Updated State)}$$

$$\mathbf{P}_k^+ = (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k) \mathbf{P}_k^- \quad \text{(Updated Covariance)}$$

Where:
- $\mathbf{H}_k = \left. \frac{\partial h}{\partial \mathbf{x}} \right\vert{}_{\mathbf{\hat{x}}_k^-}$ is the observation Jacobian.
- $\mathbf{R}$ is the measurement noise covariance matrix.

---

## 🛠️ Prerequisites & Dependencies

- **C++ Compiler**: C++17 or higher (MSVC via Visual Studio 2022 recommended)
- **Eigen 3**: Header-only library for linear algebra
- **Raylib**: Lightweight GUI/graphics library (installed via NuGet package manager or vcpkg)

---

## 🚀 Getting Started

1. **Clone the repository:**
   ```bash
   git clone [https://github.com/Coce1/KalmanFilter.git](https://github.com/Coce1/KalmanFilter.git)
