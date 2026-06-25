# Dynamics MPC Controller

ROS 2 controller package for fixed-base inverse/forward dynamics MPC using OCS2 and Pinocchio CppAD.

- The inverse dynamics MPC formulation uses [`pinocchio::rnea()`](./src/inverse_dynamics_mpc/constraint/inverse_dynamics_rnea_constraint_cppad.cpp#L50) as state-input equality constraint.
- The forward dynamics MPC formulation uses [`pinocchio::aba()`](./src/forward_dynamics_mpc/dynamics/forward_dynamics_aba_dynamics_ad.cpp#L41) as system dynamics model.

## UR5 MuJoCo Example

Single `ur5` example:
```bash
ros2 launch dynamics_mpc_controller ur5.launch.py \
  mpcControllerName:=inverse_dynamics_mpc_controller \
  mujoco_headless:=true
```

Dual `ur5` example:
```bash
ros2 launch dynamics_mpc_controller dual_ur5.launch.py \
  mpcControllerName:=inverse_dynamics_mpc_controller \
  mujoco_headless:=true
```

Usefull launch args:
```bash
mpcControllerName:=inverse_dynamics_mpc_controller | forward_dynamics_mpc_controller
mujoco_headless:=true | false
mujoco_real_time_factor:=1.0
mpcFreq:=50 # (should be integer)
mrtFreq:=1000 # (should be integer)
```

Publish a joint target:

```bash
cd <workspace_dir>/src/dynamics_mpc_controller/launch && \
python3 joint_tracking_target.py
```

For dual UR5:

```bash
cd <workspace_dir>/src/dynamics_mpc_controller/launch && \
python3 dual_arm_joint_tracking_target.py
```

Useful topics:

```bash
ros2 topic echo /mpc_observation
ros2 topic echo /mpc_policy
ros2 topic echo /estimated_ee_wrench
```

## Contact

- **Author**: Wei-Hsuan Cheng [(johnathancheng0125@gmail.com)](mailto:johnathancheng0125@gmail.com)
- **Homepage**: [wei-hsuan-cheng](https://wei-hsuan-cheng.github.io)
- **GitHub**: [wei-hsuan-cheng](https://github.com/wei-hsuan-cheng)
