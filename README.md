# Dynamics MPC Controller

ROS 2 controller package for fixed-base inverse-dynamics torque MPC using OCS2 and Pinocchio CppAD.

## UR5 MuJoCo Example

```bash
ros2 launch dynamics_mpc_controller ur5.launch.py \
  mujoco_headless:=true
```

Publish a joint target:

```bash
ros2 run dynamics_mpc_controller joint_tracking_target.py --ros-args \
  -p use_sim_time:=true
```

Inspect diagnostics:

```bash
ros2 topic hz /inverse_dynamics_mpc_policy
ros2 topic echo /estimated_ee_wrench --once
```

## Contact

- **Author**: Wei-Hsuan Cheng [(johnathancheng0125@gmail.com)](mailto:johnathancheng0125@gmail.com)
- **Homepage**: [wei-hsuan-cheng](https://wei-hsuan-cheng.github.io)
- **GitHub**: [wei-hsuan-cheng](https://github.com/wei-hsuan-cheng)
