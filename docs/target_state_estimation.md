# Target State Estimation

An Extended Kalman Filter (EKF) estimates the target's position, velocity, and acceleration from radar measurements. The state is $\mathbf{x}(t)=\left[ \mathbf{p}_ {\mathrm{T}}^{\top}(t),\mathbf{v}_ {\mathrm{T}}^{\top}(t),\mathbf{a}_ {\mathrm{T}}^{\top}(t) \right]^{\top}$ in the inertial frame. The interceptor's position, velocity, and attitude are assumed known at each measurement timestamp. The motion model for prediction is linear and the EKF approximation is needed for the nonlinear radar measurement model.

## Prediction model

The target follows a constant-acceleration model driven by zero-mean white jerk $\mathbf{w}(t)$:

$$
\dot{\mathbf{x}}(t)=
\begin{bmatrix}
\mathbf{v}_ {\mathrm{T}}(t) \\
\mathbf{a}_ {\mathrm{T}}(t) \\
\mathbf{w}(t)
\end{bmatrix}.
$$

For the interval $\Delta t$ between measurements, the exact discrete transition matrix and process-noise covariance are

$$
\mathbf{F}(\Delta t)=
\begin{bmatrix}
\mathbf{I}_3&\Delta t\,\mathbf{I}_3&\tfrac{1}{2}\Delta t^2\mathbf{I}_3 \\
\mathbf{0}&\mathbf{I}_3&\Delta t\,\mathbf{I}_3 \\
\mathbf{0}&\mathbf{0}&\mathbf{I}_3
\end{bmatrix},
$$

$$
\mathbf{Q}(\Delta t)=q_j
\begin{bmatrix}
\tfrac{\Delta t^5}{20}\mathbf{I}_3&\tfrac{\Delta t^4}{8}\mathbf{I}_3&\tfrac{\Delta t^3}{6}\mathbf{I}_3 \\
\tfrac{\Delta t^4}{8}\mathbf{I}_3&\tfrac{\Delta t^3}{3}\mathbf{I}_3&\tfrac{\Delta t^2}{2}\mathbf{I}_3 \\
\tfrac{\Delta t^3}{6}\mathbf{I}_3&\tfrac{\Delta t^2}{2}\mathbf{I}_3&\Delta t\,\mathbf{I}_3
\end{bmatrix}.
$$

Here $\mathbf{I}_3$ is the three-dimensional identity matrix and $q_j$ is the continuous white-jerk spectral density. Increasing $q_j$ allows faster changes in estimated acceleration at the expense of greater uncertainty.

The prediction step is

$$
\widehat{\mathbf{x}}_ {\mathrm{k}|\mathrm{k}-1}
=\mathbf{F}(\Delta t)\widehat{\mathbf{x}}_ {\mathrm{k}-1|\mathrm{k}-1},
$$

$$
\begin{aligned}
\mathbf{P}_ {\mathrm{k}|\mathrm{k}-1}
&=\mathbf{F}(\Delta t)\mathbf{P}_ {\mathrm{k}-1|\mathrm{k}-1}\mathbf{F}^{\top}(\Delta t) \\
&\quad+\mathbf{Q}(\Delta t).
\end{aligned}
$$

## Measurement model

The measurement is $\mathbf{z}_ {\mathrm{k}} = \left[ \rho_ {\mathrm{k}}, \dot{\rho}_ {\mathrm{k}}, \lambda_ {\mathrm{az},\mathrm{k}}, \lambda_ {\mathrm{el},\mathrm{k}} \right]^{\top}$. Define $\mathbf{r}=\mathbf{p}_ {\mathrm{T}}-\mathbf{p}_ {\mathrm{I}}$, $\mathbf{v}_ r=\mathbf{v}_ {\mathrm{T}}-\mathbf{v}_ {\mathrm{I}}$, and $\mathbf{r}_ {\mathrm{B}}=\mathbf{R}_ {\mathrm{WB}}^{\top}\mathbf{r}$, where $\mathbf{R}_ {\mathrm{WB}}$ maps body-frame vectors into the inertial frame. With $\rho=\|\mathbf{r}\|$ and $s=\sqrt{r_ {\mathrm{B},x}^2+r_ {\mathrm{B},y}^2}$, the observation model is

$$
\mathbf{z}_ {\mathrm{k}}=\mathbf{h}(\mathbf{x}_ {\mathrm{k}})+\boldsymbol{\nu}_ {\mathrm{k}},
$$

$$
\mathbf{h}(\mathbf{x})=
\begin{bmatrix}
\rho \\
\mathbf{r}^{\top}\mathbf{v}_r/\rho \\
\text{atan2}(r_ {\mathrm{B},y},r_ {\mathrm{B},x}) \\
\text{atan2}(r_ {\mathrm{B},z},s)
\end{bmatrix},
$$

$$
\boldsymbol{\nu}_ {\mathrm{k}}\sim\mathcal{N}(\mathbf{0},\mathbf{R}).
$$

The known interceptor state is an implicit input to $\mathbf{h}$. The diagonal covariance $\mathbf{R}$ contains the assumed range, range-rate, azimuth, and elevation noise variances. These are configured independently of the sensor model's actual noise variances.

### Measurement Jacobian

The observation Jacobian $\mathbf{H}_ {\mathrm{k}}=\left.\partial\mathbf{h}/\partial\mathbf{x}\right|_ {\widehat{\mathbf{x}}_ {\mathrm{k}|\mathrm{k}-1}}$ is computed analytically. With $\mathbf{e}_r=\mathbf{r}/\rho$, its block structure is

$$
\mathbf{H}=
\begin{bmatrix}
\mathbf{e}_r^{\top}&\mathbf{0}^{\top}&\mathbf{0}^{\top} \\
\mathbf{g}_ {\dot\rho}^{\top}&\mathbf{e}_r^{\top}&\mathbf{0}^{\top} \\
\mathbf{g}_ {\mathrm{az}}^{\top}&\mathbf{0}^{\top}&\mathbf{0}^{\top} \\
\mathbf{g}_ {\mathrm{el}}^{\top}&\mathbf{0}^{\top}&\mathbf{0}^{\top}
\end{bmatrix},
$$

where the position gradients are

$$
\mathbf{g}_ {\dot\rho}=\frac{(\mathbf{I}_3-\mathbf{e}_r\mathbf{e}_r^{\top})\mathbf{v}_r}{\rho},
$$

$$
\mathbf{g}_ {\mathrm{az}}=\mathbf{R}_ {\mathrm{WB}}
\begin{bmatrix}
-r_ {\mathrm{B},y}/s^2 \\
r_ {\mathrm{B},x}/s^2 \\
0
\end{bmatrix},
$$

$$
\mathbf{g}_ {\mathrm{el}}=\mathbf{R}_ {\mathrm{WB}}
\begin{bmatrix}
-r_ {\mathrm{B},x}r_ {\mathrm{B},z}/(\rho^2s) \\
-r_ {\mathrm{B},y}r_ {\mathrm{B},z}/(\rho^2s) \\
s/\rho^2
\end{bmatrix}.
$$

Acceleration is not observed directly; it is inferred over successive measurements through the motion model and state cross-covariances. Small denominator floors regularize the implementation near zero range and the body vertical axis.

### Measurement correction

The innovation, innovation covariance, and Kalman gain are

$$
\mathbf{y}_ {\mathrm{k}}=\mathbf{z}_ {\mathrm{k}}-\mathbf{h}(\widehat{\mathbf{x}}_ {\mathrm{k}|\mathrm{k}-1}),
$$

$$
\mathbf{S}_ {\mathrm{k}}=\mathbf{H}_ {\mathrm{k}}\mathbf{P}_ {\mathrm{k}|\mathrm{k}-1}\mathbf{H}_ {\mathrm{k}}^{\top}+\mathbf{R},
$$

$$
\mathbf{K}_ {\mathrm{k}}=\mathbf{P}_ {\mathrm{k}|\mathrm{k}-1}\mathbf{H}_ {\mathrm{k}}^{\top}\mathbf{S}_ {\mathrm{k}}^{-1}.
$$

The estimate and covariance are corrected as

$$
\widehat{\mathbf{x}}_ {\mathrm{k}|\mathrm{k}}
=\widehat{\mathbf{x}}_ {\mathrm{k}|\mathrm{k}-1}+\mathbf{K}_ {\mathrm{k}}\mathbf{y}_ {\mathrm{k}},
$$

$$
\mathbf{P}_ {\mathrm{k}|\mathrm{k}}
=(\mathbf{I}_9-\mathbf{K}_ {\mathrm{k}}\mathbf{H}_ {\mathrm{k}})\mathbf{P}_ {\mathrm{k}|\mathrm{k}-1}.
$$

## Initialization 

The first radar measurement initializes target position in the inertial frame, with zero velocity and acceleration and configurable initial covariance. 
