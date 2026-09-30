# Target Trajectory Generator

The trajectory generator provides deterministic target position, velocity, and acceleration in the right-handed, z-up inertial frame. Each maneuver is evaluated analytically at the requested time.

## Maneuver plane

Circular, figure-eight, and helical trajectories use a configurable center $\mathbf{c}$, plane normal $\mathbf{b}$, and reference direction $\mathbf{d}$. These define an orthonormal basis:

$$
\mathbf{e}_3=\frac{\mathbf{b}}{\|\mathbf{b}\|}.
$$

$$
\mathbf{e}_1=
\frac{\mathbf{d}-(\mathbf{d}^{\top}\mathbf{e}_3)\mathbf{e}_3}
{\|\mathbf{d}-(\mathbf{d}^{\top}\mathbf{e}_3)\mathbf{e}_3\|}.
$$

$$
\mathbf{e}_2=\mathbf{e}_3\times\mathbf{e}_1.
$$

The maneuver plane is spanned by $\mathbf{e}_1$ and $\mathbf{e}_2$, while $\mathbf{e}_3$ defines the helix axis. The normal must be nonzero and the reference direction must not be parallel to it. Choosing $\mathbf{e}_3=\left[ 0,0,1 \right]^{\top}$ gives a horizontal plane; other orientations produce tilted maneuvers.

## Constant velocity

Given initial position $\mathbf{p}_0$ and velocity $\mathbf{v}_0$, the target follows a straight line:

$$
\mathbf{p}_ {\mathrm{T}}(t)=\mathbf{p}_0+t\mathbf{v}_0.
$$

$$
\mathbf{v}_ {\mathrm{T}}(t)=\mathbf{v}_0.
$$

$$
\mathbf{a}_ {\mathrm{T}}(t)=\mathbf{0}.
$$

## Circular turn

A circle is parameterized by speed $v$ and signed lateral load factor $n$. The radius is $r_c=v^2/(|n|g)$ and angular rate is $\omega=ng/v$, with phase $\theta(t)=\omega t$. Define the radial displacement

$$
\mathbf{r}_c(t)=r_c
\left[\cos\theta(t)\,\mathbf{e}_1+\sin\theta(t)\,\mathbf{e}_2\right].
$$

The target state is

$$
\mathbf{p}_ {\mathrm{T}}(t)=\mathbf{c}+\mathbf{r}_c(t).
$$

$$
\mathbf{v}_ {\mathrm{T}}(t)=r_c\omega
\left[-\sin\theta(t)\,\mathbf{e}_1+\cos\theta(t)\,\mathbf{e}_2\right].
$$

$$
\mathbf{a}_ {\mathrm{T}}(t)=-\omega^2\mathbf{r}_c(t).
$$

Speed is constant and acceleration is centripetal, with magnitude $|n|g$. The sign of $n$ sets the turn direction, and the initial position is $\mathbf{c}+r_c\mathbf{e}_1$. Here $n$ specifies lateral acceleration rather than the total lift load factor of a gravity-balanced aircraft; speed must be positive and $n$ nonzero.

## Figure-eight maneuver

The figure-eight is a Gerono lemniscate with tip-to-tip length $L$, width $W$, and phase $\theta(t)=\omega t$. Its coordinates along the maneuver plane have a frequency ratio of $2:1$:

$$
\mathbf{p}_ {\mathrm{T}}(t)
=\mathbf{c}+\frac{L}{2}\sin\theta(t)\,\mathbf{e}_1+\frac{W}{2}\sin(2\theta(t))\,\mathbf{e}_2.
$$

$$
\mathbf{v}_ {\mathrm{T}}(t)
=\frac{L\omega}{2}\cos\theta(t)\,\mathbf{e}_1+W\omega\cos(2\theta(t))\,\mathbf{e}_2.
$$

$$
\mathbf{a}_ {\mathrm{T}}(t)
=-\frac{L\omega^2}{2}\sin\theta(t)\,\mathbf{e}_1-2W\omega^2\sin(2\theta(t))\,\mathbf{e}_2.
$$

The target starts at the center and completes a full figure eight in $2\pi/|\omega|$ for nonzero $\omega$. The phase rate is constant, but speed and acceleration magnitude vary along the trajectory.

## Helical maneuver

A helix combines a circular turn with uniform motion along $\mathbf{e}_ 3$. Given total speed $v$ and climb angle $\gamma$, the in-plane speed is $v_ {\perp}=v\cos\gamma$ and axial speed is $v_ {\parallel}=v\sin\gamma$. Use the circular displacement above with radius $r_c=v_ {\perp}^2/(|n|g)$ and angular rate $\omega=ng/v_ {\perp}$:

$$
\mathbf{p}_ {\mathrm{T}}(t)
=\mathbf{c}+\mathbf{r}_c(t)+v_ {\parallel}t\,\mathbf{e}_3.
$$

$$
\mathbf{v}_ {\mathrm{T}}(t)
=r_c\omega\left[-\sin\theta(t)\,\mathbf{e}_1
+\cos\theta(t)\,\mathbf{e}_2\right]+v_ {\parallel}\mathbf{e}_3.
$$

$$
\mathbf{a}_ {\mathrm{T}}(t)=-\omega^2\mathbf{r}_c(t).
$$

The radius and total speed remain constant, and acceleration has no axial component. For a vertical axis, positive $\gamma$ produces a climb; for a tilted axis, $\gamma$ describes inclination relative to the maneuver plane rather than the inertial horizontal plane. Zero climb angle recovers the circular trajectory, while zero in-plane speed or zero load factor is invalid for this parameterization.

These trajectories prescribe target motion directly; they do not simulate target actuators, drag, or flight-envelope constraints.
