# Simulation

The interceptor is simulated as a three-degree-of-freedom point mass in a right-handed, z-up inertial frame. The model describes translational motion under thrust, drag, gravity, and commanded lift. Rotational dynamics and low-level attitude control are omitted. Load factor and bank commands act directly on the velocity direction.

## Continuous dynamics

The state is $\mathbf{x}(t)=\left[ p_x(t),p_y(t),p_z(t),v(t),\psi(t),\gamma(t) \right]^{\top}$ and the control is $\mathbf{u}(t)=\left[ T(t),n(t),\alpha(t) \right]^{\top}$. Here $v$ is speed, $\psi$ is heading from inertial $+x$ toward $+y$, and $\gamma$ is flight-path angle, positive during climb. The inputs are thrust $T$, load factor $n=L/(mg)$, and bank angle $\alpha$.

$$
\dot{\mathbf{x}}(t)=\mathbf{f}(\mathbf{x}(t),\mathbf{u}(t))=
\begin{bmatrix}
v\cos\gamma\cos\psi \\
v\cos\gamma\sin\psi \\
v\sin\gamma \\
\dfrac{T-D(v)}{m}-g\sin\gamma \\
\dfrac{ng\sin\alpha}{v\cos\gamma} \\
\dfrac{g(n\cos\alpha-\cos\gamma)}{v}
\end{bmatrix}.
$$

Drag is modeled as $D(v)=\tfrac{1}{2}\rho_ {\mathrm{air}}AC_Dv^2$, with constant mass $m$, air density $\rho_ {\mathrm{air}}$, reference area $A$, and drag coefficient $C_D$.

## Discretization and limits

The standalone simulation uses fourth-order Runge–Kutta integration, with the command held constant during each timestep $\Delta t$. Forward Euler and Heun integration are also available. Each update first clips the command to the actuator limits, integrates the dynamics, and then clips the resulting state to the flight envelope.

## Cartesian state and attitude

The simulator exposes position and velocity in the inertial frame to the other components. Velocity is reconstructed from speed and direction as

$$
\mathbf{v}_ {\mathrm{I}}=
v\begin{bmatrix}
\cos\gamma\cos\psi \\
\cos\gamma\sin\psi \\
\sin\gamma
\end{bmatrix}.
$$

Body attitude is derived from heading, flight-path angle, and the applied bank command, assuming zero sideslip and angle of attack. The body-to-inertial rotation is

$$
\mathbf{R}_ {\mathrm{WB}}
=\mathbf{R}_z(\psi)\mathbf{R}_y(-\gamma)\mathbf{R}_x(\alpha).
$$

The negative pitch angle follows the z-up convention, aligning the body's forward axis with the velocity. This rotation is stored as a unit quaternion for use by the sensor model. Bank is imposed directly rather than integrated as an attitude state. 
