# Dynamics MPC Formulations

This package contains fixed-base joint space dynamics MPC formulations using OCS2 and Pinocchio:

- inverse dynamics MPC with Pinocchio RNEA (recursive Newton-Euler algorithm)
- forward dynamics MPC with Pinocchio ABA (articulated-body algorithm)

Let the manipulator have $n$ actuated joints. Both formulations use the joint space state

$$
x =
\begin{bmatrix}
q \\
v
\end{bmatrix}
\in \mathbb{R}^{2n},
$$

where $q$ is joint position and $v = \dot q$ is joint velocity.

## Generic OCP

Both controllers solve a finite-horizon optimal control problem

$$
\begin{aligned}
\min_{x(\cdot),u(\cdot)} \quad
& \int_{t_0}^{t_0+T}
\left[
\ell_x(x(t), x_\mathrm{ref}(t))
+ \ell_u(u(t), u_\mathrm{ref}(t))
\right] dt \\
\text{s.t.} \quad
& \dot x(t) = f(x(t), u(t)), \\
& h(x(t), u(t)) = 0, \\
& \underline{x} \le x(t) \le \overline{x}, \\
& \underline{u} \le u(t) \le \overline{u}, \\
& x(t_0) = x_0 .
\end{aligned}
$$

The equality constraint $h(x,u)=0$ is formulation-dependent. The state and input box constraints are active only when enabled in the YAML parameters.

The current formulation uses running costs only; there is no separate terminal cost term.

## Joint Tracking Cost

The joint tracking running cost is

$$
\ell_x(x, x_\mathrm{ref})
=
\frac{1}{2}
(x - x_\mathrm{ref})^\top
Q(t)
(x - x_\mathrm{ref}),
$$

with

$$
Q(t) =
\begin{bmatrix}
Q_q(t) & 0 \\
0 & Q_v(t)
\end{bmatrix}.
$$

Depending on the target command type:

$$
x_\mathrm{ref} =
\begin{cases}
\begin{bmatrix}q_\mathrm{ref} \\ 0\end{bmatrix},
& \text{joint position tracking}, \\ \\
\begin{bmatrix}0 \\ v_\mathrm{ref}\end{bmatrix},
& \text{joint velocity tracking}, \\ \\
\begin{bmatrix}q_\mathrm{ref} \\ v_\mathrm{ref}\end{bmatrix},
& \text{joint position + velocity tracking}.
\end{cases}
$$

For position-only tracking, $Q_v = 0$. For velocity-only tracking, $Q_q = 0$.

## Input Tracking Cost

The input tracking running cost is

$$
\ell_u(u, u_\mathrm{ref})
=
\frac{1}{2}
(u - u_\mathrm{ref})^\top
R
(u - u_\mathrm{ref}).
$$

If no target input is provided with the expected dimension, the input reference is zero.

## Inverse Dynamics MPC

### Input

Without end-effector wrench in RNEA:

$$
u =
\begin{bmatrix}
a \\
\tau
\end{bmatrix}
\in \mathbb{R}^{2n},
$$

where $a=\ddot q$ is optimized joint acceleration and $\tau$ is optimized joint torque.

With end-effector wrench in RNEA:

$$
u =
\begin{bmatrix}
a \\
\tau \\
F_\mathrm{ee}
\end{bmatrix}
\in \mathbb{R}^{2n+6},
$$

where

$$
F_\mathrm{ee} =
\begin{bmatrix}
f_\mathrm{ee} \\
m_\mathrm{ee}
\end{bmatrix}
\in \mathbb{R}^{6}
$$

is the spatial wrench expressed at the selected end-effector frame.

### Dynamics

The inverse dynamics MPC uses kinematic state dynamics:

$$
\dot x =
\begin{bmatrix}
\dot q \\
\dot v
\end{bmatrix}
=
\begin{bmatrix}
v \\
a
\end{bmatrix}.
$$

### RNEA Equality Constraint

Without end-effector wrench:

$$
h_\mathrm{RNEA}(x,u)
=
\operatorname{RNEA}(q,v,a) - \tau
= 0 .
$$

With end-effector wrench:

$$
h_\mathrm{RNEA+wrench}(x,u)
=
\operatorname{RNEA}(q,v,a)
- J_\mathrm{ee}(q)^\top F_\mathrm{ee}
- \tau
= 0 .
$$

Here $J_\mathrm{ee}(q)\in\mathbb{R}^{6\times n}$ is the end-effector frame Jacobian.

### End-Effector Wrench Tracking Constraint

When wrench tracking is enabled, the wrench is enforced as a hard equality:

$$
h_F(x,u,t)
=
F_\mathrm{ee} - F_{\mathrm{ee,ref}}(t)
= 0 .
$$

$F_{\mathrm{ee,ref}}(t)$ is read from the target input trajectory when provided.
If no wrench reference is provided, it defaults to zero.

### Input Cost

Without end-effector wrench:

$$
R =
\begin{bmatrix}
R_a & 0 \\
0 & R_\tau
\end{bmatrix}.
$$

With end-effector wrench:

$$
R =
\begin{bmatrix}
R_a & 0 & 0 \\
0 & R_\tau & 0 \\
0 & 0 & R_F
\end{bmatrix}.
$$

## Forward Dynamics MPC

### Input

The forward dynamics MPC optimizes joint torque directly:

$$
u = \tau \in \mathbb{R}^{n}.
$$

### Dynamics

The forward dynamics MPC uses Pinocchio ABA as system dynamics:

$$
\dot x =
\begin{bmatrix}
\dot q \\
\dot v
\end{bmatrix}
=
\begin{bmatrix}
v \\
\operatorname{ABA}(q,v,\tau)
\end{bmatrix}.
$$

There is no RNEA equality constraint in the forward dynamics formulation.

### Input Cost

$$
R = R_\tau .
$$

## Box Constraints

State box constraints are

$$
\underline{q} \le q \le \overline{q},
\qquad
\underline{v} \le v \le \overline{v}.
$$

Inverse dynamics MPC input box constraints are

$$
\underline{a} \le a \le \overline{a},
\qquad
\underline{\tau} \le \tau \le \overline{\tau},
$$

and, when wrench input is present,

$$
\underline{F}_\mathrm{ee}
\le
F_\mathrm{ee}
\le
\overline{F}_\mathrm{ee}.
$$

Forward dynamics MPC input box constraints are

$$
\underline{\tau} \le \tau \le \overline{\tau}.
$$

## Moving Obstacle Avoidance

The inverse dynamics MPC also keeps the arm away from tracked moving obstacles (`ocs2.task.dynamicObstacles`), published on `/dynamic_obstacles` as `dynamics_mpc_controller/TrackedObstacleArray`, as a vision tracker would.
Each obstacle $j$ is a set of spheres with centers $c_{jk}$ in its own frame and radii $r_{jk}$, a pose $(p_j, R_j)$ at the message stamp $t_s$, and a twist $(\nu_j, \omega_j)$.

### Prediction

Over the horizon, each obstacle moves on at constant velocity ([`center()`](../src/common/constraint/dynamic_obstacles.cpp#L25)):

$$
s_{jk}(t) = p_j + \Delta t\, \nu_j + \exp\!\left([\omega_j]_\times \Delta t\right) R_j\, c_{jk},
\qquad
\Delta t = \operatorname{clamp}(t - t_s,\ 0,\ \Delta t_\mathrm{max}),
$$

with $\Delta t_\mathrm{max}$ = `maxPredictionTime`.
The radius grows with the tracker's 1-sigma position uncertainty $\sigma_j$ ([L189](../src/common/constraint/dynamic_obstacles.cpp#L189)):

$$
\rho_{jk} = r_{jk} + \kappa\, \sigma_j,
$$

with $\kappa$ = `positionStdScale`.

### Soft Constraint

For every collision primitive $g$ (cylinder, box, sphere, ...) of the URDF links in `dynamicObstacles.links`, and every obstacle sphere $i = (j, k)$, the clearance is ([L147](../src/common/constraint/dynamic_obstacle_soft_constraint.cpp#L147))

$$
h_{gi}(q, t) = d\big(g(q),\ \mathcal{B}(s_i(t), \rho_i)\big) - d_\mathrm{min},
$$

where $d$ is the signed coal distance between the posed primitive and the ball, and $d_\mathrm{min}$ = `minimumDistance`.
The running cost adds the relaxed log barrier $B_{\mu,\delta}$ of every clearance ([L148](../src/common/constraint/dynamic_obstacle_soft_constraint.cpp#L148)):

$$
\ell_\mathrm{obs}(x, t) = \sum_{g} \sum_{i} w_i(t_0)\, B_{\mu,\delta}\big(h_{gi}(q, t)\big),
\qquad
B_{\mu,\delta}(h) =
\begin{cases}
-\mu \ln h, & h > \delta, \\
\mu \left( \dfrac{1}{2} \left( \left( \dfrac{h - 2\delta}{\delta} \right)^2 - 1 \right) - \ln \delta \right), & h \le \delta,
\end{cases}
$$

the same barrier as `selfCollision` with `implementation: rbf`.
The weight $w_i(t_0) = \operatorname{clamp}((t_0 - t_\mathrm{seen})/T_\mathrm{fade},\ 0,\ 1)$ ramps a newly seen obstacle in over $T_\mathrm{fade}$ = `fadeInTime` ([L172](../src/common/constraint/dynamic_obstacles.cpp#L172)), so the plan does not jump when it appears.

### Derivatives

Let $p_g$ be the witness point of the primitive (the coal nearest point on $g$), $n_g$ the coal normal from the primitive to the ball, and $J_{p_g}(q)$ the Jacobian of $p_g$ as a point fixed to the primitive's joint:

$$
J_{p_g} = J_v - [p_g - o_\mathrm{joint}]_\times J_\omega
$$

from the joint's `LOCAL_WORLD_ALIGNED` Jacobian $[J_v; J_\omega]$.
Moving the primitive by $\delta p_g$ changes the distance by $-n_g^\top \delta p_g$, also when they penetrate, so ([L164](../src/common/constraint/dynamic_obstacle_soft_constraint.cpp#L164))

$$
\frac{\partial h_{gi}}{\partial q} = -\,n_g^\top J_{p_g}(q).
$$

The gradient and the Gauss-Newton Hessian are those of OCS2's soft constraints ([L165-L166](../src/common/constraint/dynamic_obstacle_soft_constraint.cpp#L165-L166)):

$$
\nabla_q \ell_\mathrm{obs} = \sum_{g,i} w_i\, B'(h_{gi})\, \frac{\partial h_{gi}}{\partial q}^{\top},
\qquad
\nabla^2_q \ell_\mathrm{obs} \approx \sum_{g,i} w_i\, B''(h_{gi})\, \frac{\partial h_{gi}}{\partial q}^{\top} \frac{\partial h_{gi}}{\partial q}.
$$

### Synchronization

The obstacles change between solves but never during one.
The subscription stores each message; right before every solve, an OCS2 `SolverSynchronizedModule` takes a snapshot at the MPC start time $t_0$, assigning slots, weights and inflated radii, and the cost (and its per-thread clones) reads only that snapshot.
An obstacle keeps its slot while its id stays in the messages; beyond `maxObstacles` the obstacles nearest the base are kept, and no message for `timeout` seconds clears them all.

## Solvers

The controller selects the OCS2 MPC default solvers (DDP and SQP)

$$
\texttt{solverType} \in \{\texttt{sqp}, \texttt{ddp}\}.
$$

For SQP:

$$
\text{solver} = \operatorname{SqpMpc}.
$$

For DDP:

$$
\text{solver} = \operatorname{GaussNewtonDDP\_MPC}.
$$
