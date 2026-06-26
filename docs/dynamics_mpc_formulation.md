# Dynamics MPC Formulations

This package contains fixed-base joint-space dynamics MPC formulations using OCS2 and Pinocchio:

- inverse dynamics MPC with Pinocchio RNEA (recursive Newton-Euler algorithm)
- forward dynamics MPC with Pinocchio ABA (articulated-body algorithm)

Let the manipulator have $n$ actuated joints. Both formulations use the joint-space state

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

For zero-wrench tracking:

$$
F_{\mathrm{ee,ref}}(t) = 0.
$$

For commanded wrench tracking:

$$
F_{\mathrm{ee,ref}}(t)
$$

is read from the target input trajectory.

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

## Solvers

The controller selects the OCS2 MPC solver from the YAML field

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

The current UR5 and dual-UR5 example configurations use SQP.
