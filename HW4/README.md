# Homework 4: Chart Results and Simulation Videos

Currently using GIFs. If they don't play automatically, please wait a few seconds or refresh the GitHub page.

---

## Problem 1: Cart-Pole Control

### a. LQR control (infinite-horizon continuous-time)

#### i) Q = diag(10, 10, 10, 10); R = 1

<p align="center">
  <img src="assets/video_lqr_case1.gif" alt="LQR Case 1" width="66%">
</p>
<p align="center">
  <em>LQR Control Simulation (Case 1)</em>
</p>

<p align="center">
  <img src="assets/LQR_Q10_10_10_10_R1.png" alt="LQR Case 1 Plot" width="66%">
</p>
<p align="center">
  <em>LQR Control States (Case 1)</em>
</p>

#### ii) Q = diag(100, 0.01, 100, 0.01); R = 10

<p align="center">
  <img src="assets/video_lqr_case2.gif" alt="LQR Case 2" width="66%">
</p>
<p align="center">
  <em>LQR Control Simulation (Case 2)</em>
</p>

<p align="center">
  <img src="assets/LQR_Q100_001_100_001_R10.png" alt="LQR Case 2 Plot" width="66%">
</p>
<p align="center">
  <em>LQR Control States (Case 2)</em>
</p>

#### iii) Q = diag(100, 0.01, 100, 0.01); R = 1

<p align="center">
  <img src="assets/video_lqr_case3.gif" alt="LQR Case 3" width="66%">
</p>
<p align="center">
  <em>LQR Control Simulation (Case 3)</em>
</p>

<p align="center">
  <img src="assets/LQR_Q100_001_100_001_R1.png" alt="LQR Case 3 Plot" width="66%">
</p>
<p align="center">
  <em>LQR Control States (Case 3)</em>
</p>

### b. QP control

<p align="center">
  <img src="assets/video_qp.gif" alt="QP Control" width="66%">
</p>
<p align="center">
  <em>QP Control Simulation</em>
</p>

<p align="center">
  <img src="assets/QP control.png" alt="QP Control Plot" width="66%">
</p>
<p align="center">
  <em>QP Control States</em>
</p>

### c. MPC control

<p align="center">
  <img src="assets/video_mpc.gif" alt="MPC Control" width="66%">
</p>
<p align="center">
  <em>MPC Control Simulation</em>
</p>

<p align="center">
  <img src="assets/MPC control.png" alt="MPC Control Plot" width="66%">
</p>
<p align="center">
  <em>MPC Control States</em>
</p>

---

## Problem 2

### a. QP Control

<p align="center">
  <img src="assets/robot_qp.gif" alt="Robot QP Control" width="66%">
</p>
<p align="center">
  <em>Robot QP Control Simulation</em>
</p>

<p align="center">
  <img src="assets/QP_xytheta.png" alt="QP x, y, theta Plot" width="66%">
</p>
<p align="center">
  <em>QP Control States (x, y, theta)</em>
</p>

<p align="center">
  <img src="assets/QP_Torque.png" alt="QP Torque Plot" width="66%">
</p>
<p align="center">
  <em>QP Control Torques</em>
</p>
