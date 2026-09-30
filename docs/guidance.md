# Guidance

The controllers receive the interceptor and target Cartesian states and return the command

$$
\mathbf{u}_{\mathrm{cmd}} =
\begin{bmatrix} T & n & \alpha \end{bmatrix}^{\top},
$$

where $T$ is thrust, $n$ is load factor, and $\alpha$ is bank angle. Positions, velocities, and accelerations are expressed in the right-handed, z-up inertial frame. Subscripts $\mathrm{I}$ and $\mathrm{T}$ denote interceptor and target; target states may come from ground truth or estimated from an EKF. Heading is $\psi$, flight-path angle is $\gamma$, and scalar speed is $v=\|\mathbf{v}_{\mathrm{I}}\|$.

### Thrust control

During boost, the controller commands fixed thrust with $n=1$ and $\alpha=0$ until the speed reaches the threshold $v_{\mathrm{thr}}$. The thrust command is

$$
T = \begin{cases} T_ {\max}, & v < v_ {\mathrm{thr}} \\
T_ {\mathrm{trim}}, & v \geq v_ {\mathrm{thr}}  \\
\end{cases}
$$


Here $T_{\max}$ denotes the configured boost thrust and $T_{\mathrm{trim}}$ the trim thrust that compensates drag $D(v)=\frac{1}{2}\rho_{\mathrm{air}} A C_D v^2$ and the component of gravity along the velocity:

$$
F_{\mathrm{trim}}=D(v)+mg\sin\gamma.
$$

## Proportional Navigation Control

Proportional navigation commands a transverse acceleration from the line-of-sight (LOS) rotation and closing speed. Define relative position, relative velocity, and range as

$$
\mathbf{r}=\mathbf{p}_ {\mathrm{T}}-\mathbf{p}_ {\mathrm{I}},
\qquad
\mathbf{v}_ {r}=\mathbf{v}_{\mathrm{T}}-\mathbf{v}_{\mathrm{I}},
\qquad
\rho=\|\mathbf{r}\|.
$$

The closing speed and LOS angular velocity are

$$
V_c=-\dot{\rho}=-\frac{\mathbf{r}^{\top}\mathbf{v}_ {r}}{\rho},
\qquad
\boldsymbol{\omega}_ {\mathrm{LOS}}
=\frac{\mathbf{r}\times\mathbf{v}_{r}}{\rho^2}.
$$

The implemented three-dimensional PN law is

$$
\mathbf{a}_ {\mathrm{PN}}
=N_{\mathrm{PN}}V_c
\left(\boldsymbol{\omega}_ {\mathrm{LOS}}\times
\frac{\mathbf{v}_ {\mathrm{I}}}{v}\right),
$$

where $N_{\mathrm{PN}}$ is the navigation gain. The command is perpendicular to the interceptor velocity and steers its direction. A constant LOS direction gives zero PN acceleration. Small positive floors on range and speed protect the implementation against division by zero.

### Control allocation

The simulator accepts load factor and bank angle rather than an inertial acceleration. Resolve the PN command along the transverse directions associated with increasing heading and flight-path angle:

$$
\mathbf{e}_ {\psi}=
\begin{bmatrix}
-\sin\psi \\
\cos\psi \\
0
\end{bmatrix},
\qquad
\mathbf{e}_ {\gamma}=
\begin{bmatrix}
-\sin\gamma\cos\psi \\
-\sin\gamma\sin\psi \\
\cos\gamma
\end{bmatrix},
\qquad
a_{\psi}=\mathbf{e}_ {\psi}^{\top}\mathbf{a}_ {\mathrm{PN}},
\quad
a_{\gamma}=\mathbf{e}_ {\gamma}^{\top}\mathbf{a}_{\mathrm{PN}}.
$$

The transverse dynamics satisfy

$$
a_{\psi}=ng\sin\alpha,
\qquad
a_{\gamma}=g(n\cos\alpha-\cos\gamma).
$$

Inverting these relations gives the unsaturated commands

$$
n=\frac{\sqrt{a_{\psi}^{2}+(a_{\gamma}+g\cos\gamma)^2}}{g},
\qquad
\alpha=\arctan2
\left(a_{\psi},\,a_{\gamma}+g\cos\gamma\right).
$$

## Predictive Interception Control

The predictive controller solves a finite-horizon optimal-control problem using an iterative Linear-Quadratic-Regulator (iLQR) in a receding-horizon fashion to provide optimal load factor and bank angle commands. The velocity is assumed to be held constant along the horizon by the thrust control law.

### Prediction model

The internal model retains the simulator's position and direction dynamics but holds speed fixed at its current measured value over each optimization horizon. The state is $\mathbf{x}(t)=[p_x(t),p_y(t),p_z(t),\psi(t),\gamma(t)]^{\top}$ and the control is $\mathbf{u}(t)=[n(t),\alpha(t)]^{\top}$. The continuous dynamics are

$$
\dot{\mathbf{x}}(t)=\mathbf{f}(\mathbf{x}(t),\mathbf{u}(t))=
\begin{bmatrix}
v\cos\gamma\cos\psi\\
v\cos\gamma\sin\psi\\
v\sin\gamma\\
\dfrac{ng\sin\alpha}{v\cos\gamma}\\
\dfrac{g(n\cos\alpha-\cos\gamma)}{v}
\end{bmatrix}.
$$

Speed and $\cos\gamma$ denominators are regularized near their singularities. The system is discretized using Heun's method; forward Euler and fourth-order Runge–Kutta are also available as integration policies. With prediction timestep $h$, configured separately from the simulation timestep, the discrete dynamics are

$$
\mathbf{x}_ {\mathrm{k}+1}=\Phi_h(\mathbf{x}_ {\mathrm{k}},\mathbf{u}_{\mathrm{k}}).
$$

To penalize control changes, define the augmented state $\widetilde{\mathbf{x}}_ {\mathrm{k}}=\[ \mathbf{x}_ {\mathrm{k}}^{\top},\mathbf{u}_ {\mathrm{k}-1}^{\top} \]^{\top}$, which includes the previous command. Its transition is

$$
\widetilde{\mathbf{x}}_ {\mathrm{k}+1}=
\widetilde{\Phi}_ h(\widetilde{\mathbf{x}}_ {\mathrm{k}},\mathbf{u}_ {\mathrm{k}})=
\begin{bmatrix}
\Phi_h(\mathbf{x}_ {\mathrm{k}},\mathbf{u}_ {\mathrm{k}})\\
\mathbf{u}_{\mathrm{k}}
\end{bmatrix}.
$$

The dynamics Jacobians required by iLQR, $\mathbf{A}_ {\mathrm{k}}=\partial\widetilde{\Phi}_ h/\partial\widetilde{\mathbf{x}}_ {\mathrm{k}}$ and $\mathbf{B}_ {\mathrm{k}}=\partial\widetilde{\Phi}_ h/\partial\mathbf{u}_{\mathrm{k}}$, are efficiently evaluated along the nominal trajectory using automatic differentiation, yielding derivatives accurate to floating-point precision.

### Target prediction

Target predictions along the horizon use constant acceleration, initialized from the target state supplied at the current guidance update:

$$
\widehat{\mathbf{p}}_{\mathrm{T},\mathrm{k}}
=\mathbf{p}_{\mathrm{T},0}+t_{\mathrm{k}}\mathbf{v}_{\mathrm{T},0}
+\frac{1}{2}t_{\mathrm{k}}^2\mathbf{a}_{\mathrm{T},0},
\qquad t_{\mathrm{k}}=\mathrm{k}h.
$$

### Objective and constraints

For $N$ control stages, define the position error $\mathbf{e}_ {\mathrm{k}} = \mathbf{p}_ {\mathrm{I},\mathrm{k}}-\widehat{\mathbf{p}}_ {\mathrm{T},\mathrm{k}}$ and the control change $\Delta\mathbf{u}_ {\mathrm{k}}=\mathbf{u}_ {\mathrm{k}}-\mathbf{u}_ {\mathrm{k}-1}$. With terminal position error $\mathbf{e}_N$, the optimization problem is

$$
\begin{aligned}
\min_{\mathbf{u}_0,\ldots,\mathbf{u}_{N-1}}\quad
J &= w_f\|\mathbf{e}_N\|^2
+\sum_{\mathrm{k}=0}^{N-1}
\left[
w_r\|\mathbf{e}_{\mathrm{k}}\|^2
+\Delta\mathbf{u}_{\mathrm{k}}^{\top}
\mathbf{W}_{\Delta u}\Delta\mathbf{u}_{\mathrm{k}}
\right] \\
\text{s.t.}\quad
\widetilde{\mathbf{x}}_{\mathrm{k}+1}
&=\widetilde{\Phi}_h(\widetilde{\mathbf{x}}_{\mathrm{k}},\mathbf{u}_{\mathrm{k}}), \\
n_{\min} \le n_{\mathrm{k}} & \le n_{\max}, \\
-\alpha_{\max} \le \alpha_{\mathrm{k}} & \le\alpha_{\max}.
\end{aligned}
$$

with scaled weights $w_f$, $w_r$ and $\mathbf{W}_ {\Delta u}=\text{diag}(w_n,w_\alpha)$.

The running and terminal position penalties encourage interception, while the control-change penalty discourages abrupt commands.

### Adaptive horizon

A fixed long horizon can favor following the target beyond the encounter rather than reducing the immediate miss distance. The horizon is therefore shortened using a nominal interception time. This heuristic assumes constant target velocity and an interceptor that can instantly choose its direction while maintaining speed. It does not account for acceleration or turning limits.

Equating the distance to the future target position with the interceptor's travel distance gives the optimistically estimated time to collision $\tau$:

$$
\|\mathbf{r}+\tau\mathbf{v}_{\mathrm{T}}\|^2=v_{\mathrm{I}}^2\tau^2
\quad\Longrightarrow\quad
A\tau^2+B\tau+C=0,
$$

The horizon length is then

$$
N=\text{clip}
\left(\left\lceil\frac{\tau}{h}\right\rceil+1, N_{\min}, N_{\max}\right).
$$
