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
The discrete system state $x_k \in \mathbb{R}^4$ is defined by 2D coordinates and linear velocities:

$$x_k = \begin{bmatrix} x_k \\ y_k \\ \dot{x}_k \\ \dot{y}_k \end{bmatrix}$$

### 2. Prediction Step
Using a non-linear process model $f(\hat{x}_{k-1}^+, u_k)$ with sampling time $\Delta t$:

$$\hat{x}_k^- = f(\hat{x}_{k-1}^+, u_k)$$

$$P_k^- = F_k P_{k-1}^+ F_k^T + Q$$

**Where:**
- $F_k = \left. \frac{\partial f}{\partial x} \right\vert{}_{\hat{x}_{k-1}^+}$ is the process Jacobian matrix.
- $Q$ is the process noise covariance matrix.
- $P$ is the state estimation error covariance matrix.

### 3. Update Step
Given a measurement vector $z_k$ and observation model $h(x)$:

$$y_k = z_k - h(\hat{x}_k^-) \quad \text{(Innovation)}$$

$$S_k = H_k P_k^- H_k^T + R \quad \text{(Innovation Covariance)}$$

$$K_k = P_k^- H_k^T S_k^{-1} \quad \text{(Kalman Gain)}$$

$$\hat{x}_k^+ = \hat{x}_k^- + K_k y_k \quad \text{(Updated State)}$$

$$P_k^+ = (I - K_k H_k) P_k^- \quad \text{(Updated Covariance)}$$

**Where:**
- $H_k = \left. \frac{\partial h}{\partial x} \right\vert{}_{\hat{x}_k^-}$ is the observation Jacobian matrix.
- $R$ is the measurement noise covariance matrix.

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
