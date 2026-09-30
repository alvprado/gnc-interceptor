# Sensor Modeling

The sensor model represents a radar fixed to the interceptor's body frame. It receives the target and interceptor ground-truth states, including the interceptor attitude, and returns a timestamped measurement of range, range rate, azimuth, and elevation. The interceptor pose is assumed known, and the sensor is colocated with its position reference.

## Measurement geometry

Define the relative position $\mathbf{r}=\mathbf{p}_ {\mathrm{T}}-\mathbf{p}_ {\mathrm{I}}$ and velocity $\mathbf{v}_ r=\mathbf{v}_ {\mathrm{T}}-\mathbf{v}_ {\mathrm{I}}$ in the inertial frame. Range and range rate are

$$
\rho=\|\mathbf{r}\|,
\qquad
\dot{\rho}=\frac{\mathbf{r}^{\top}\mathbf{v}_r}{\rho}.
$$

Range rate is negative when closing and positive when separating, so the guidance closing speed is $V_c=-\dot{\rho}$. The implementation floors the denominator at $10^{-6}\,\mathrm{m}$ to avoid division by zero.

Let $\mathbf{R}_ {\mathrm{WB}}$ map body-frame vectors into the inertial frame. The relative position expressed in the sensor frame is

$$
\mathbf{r}_ {\mathrm{B}}=
\mathbf{R}_ {\mathrm{WB}}^{\top}\mathbf{r}
=\begin{bmatrix}
r_ {\mathrm{B},x}&r_ {\mathrm{B},y}&r_ {\mathrm{B},z}
\end{bmatrix}^{\top}.
$$

The code performs this rotation using the conjugate of the interceptor's attitude quaternion. Azimuth $\lambda_ {\mathrm{az}}$ is measured from body $+x$ toward $+y$, and elevation $\lambda_ {\mathrm{el}}$ is measured above the body $xy$ plane toward $+z$:

$$
\lambda_ {\mathrm{az}}=\text{atan2}(r_ {\mathrm{B},y},r_ {\mathrm{B},x}),
\qquad
\lambda_ {\mathrm{el}}=\text{atan2}
\left(r_ {\mathrm{B},z},\sqrt{r_ {\mathrm{B},x}^{2}+r_ {\mathrm{B},y}^{2}}\right).
$$

## Measurement noise

The ideal measurement vector is $\mathbf{h}_ {\mathrm{k}}=\left[ \rho_ {\mathrm{k}},\dot{\rho}_ {\mathrm{k}},\lambda_ {\mathrm{az},\mathrm{k}},\lambda_ {\mathrm{el},\mathrm{k}} \right]^{\top}$. Independent, zero-mean Gaussian noise is added to each channel:

$$
\mathbf{z}_ {\mathrm{k}}=\mathbf{h}_ {\mathrm{k}}+\boldsymbol{\nu}_ {\mathrm{k}},
\qquad
\boldsymbol{\nu}_ {\mathrm{k}}\sim\mathcal{N}(\mathbf{0},\mathbf{R}),
\qquad
\mathbf{R}=\text{diag}
\left(\sigma_ {\rho}^{2},\sigma_ {\dot{\rho}}^{2},
\sigma_ {\lambda_ {\mathrm{az}}}^{2},\sigma_ {\lambda_ {\mathrm{el}}}^{2}\right).
$$

Noise samples are independent across measurement updates.

The model does not include systematic bias, latency, field-of-view restriction, missed detections, or clutter. Each call produces one measurement and preserves the supplied simulation timestamp.
