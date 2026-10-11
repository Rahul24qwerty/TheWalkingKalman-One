# TheWalkingKalman-One
This project implements an Extended Kalman Filter (EKF) from first principles to estimate the state of an object following a nonlinear trajectory. The simulation demonstrates how nonlinear motion models and noisy measurements can be combined to estimate an object's position and motion over time.

The implementation covers the EKF's two fundamental stages:

1. Prediction: Propagates the estimated state and uncertainty using a nonlinear motion model and its Jacobian.

2. Update: Corrects the predicted state using noisy measurements, refining the estimate through measurement residuals and Kalman gain.

The simulation provides a practical exploration of nonlinear state estimation, uncertainty propagation, and the mathematical foundations of Kalman filtering.

# Simulation for Non-Linear Path Trajectory samples through Extended Kalman filter
<p align="center">
  <img src="Simulations/extended_kalman_filter_path_1.gif" width="48%" alt="EKF Simulation 1">
  <img src="Simulations/extended_kalman_filter_path_1.gif" width="48%" alt="EKF Simulation 2">
</p>

<p align="center">
  <img src="Simulations/extended_kalman_filter_path_3.gif" width="60%" alt="EKF Simulation 3">
</p>
