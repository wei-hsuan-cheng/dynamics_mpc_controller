# Dynamics MPC Controller

ROS 2 controller package for fixed-base inverse/forward dynamics MPC using [OCS2](https://github.com/wei-hsuan-cheng/ocs2_ros2.git) and [Pinocchio](https://github.com/stack-of-tasks/pinocchio.git).

- The inverse dynamics MPC formulation uses [`pinocchio::rnea()`](./src/inverse_dynamics_mpc/constraint/inverse_dynamics_rnea_with_ee_wrench_constraint_cppad.cpp#L59) as state-input equality constraint.
- The forward dynamics MPC formulation uses [`pinocchio::aba()`](./src/forward_dynamics_mpc/dynamics/forward_dynamics_aba_dynamics_ad.cpp#L41) as system dynamics model.

## Build and Install
Clone this repo and all sub-repo with vcs;
```bash
# Clone repos
cd <workspace_dir>/src
git clone https://github.com/wei-hsuan-cheng/dynamics_mpc_controller.git -b main
vcs import < dynamics_mpc_controller.repos # clone in the same directory as dynamics_mpc_controller

# Install by resdep
cd <workspace_dir>
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

First download `mujoco` pre-built library (details in [`mujoco_ros2_control`](https://github.com/wei-hsuan-cheng/mujoco_ros2_control.git)):
```bash
cd <your_path>
# Check x86_64 or aarch64
wget -O mujoco-3.3.7-linux-x86_64.tar.gz \
  https://github.com/google-deepmind/mujoco/releases/download/3.3.7/mujoco-3.3.7-linux-x86_64.tar.gz && \
tar -xzf mujoco-3.3.7-linux-x86_64.tar.gz
export MUJOCO_DIR=<your_path>/mujoco-3.x.x # e.g. mujoco-3.3.7 (depends on your own version)
```

Then build all pkgs up-to [`dynamics_mpc_controller`](https://github.com/wei-hsuan-cheng/dynamics_mpc_controller.git):
```bash
cd <workspace_dir>
export CMAKE_BUILD_PARALLEL_LEVEL=2 && \
export MAKEFLAGS=-j2 && \
export NINJAFLAGS=-j2 && \
colcon build --symlink-install \
  --packages-up-to dynamics_mpc_controller \
  --executor sequential --parallel-workers 2 \
  --cmake-force-configure \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release && \
  . install/setup.bash
```

## Run MuJoCo Example

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

Useful launch args:
```bash
mpcControllerName:=inverse_dynamics_mpc_controller | forward_dynamics_mpc_controller
mujoco_headless:=true | false
mujoco_real_time_factor:=1.0
mpcFreq:=50 # (should be integer)
mrtFreq:=1000 # (should be integer)
```

Useful topics:
```bash
# MPC-related
ros2 topic echo /mpc_targets
ros2 topic echo /mpc_observation
ros2 topic echo /mpc_policy

# Momemtum observed-based external EE wrench estimation
ros2 topic echo /estimated_ee_wrench
```

## Joint Target Publisher

Single `ur5` sample code:
```bash
cd <workspace_dir>/src/dynamics_mpc_controller/launch && \
python3 joint_tracking_target.py
```

Dual `ur5` sample code:
```bash
cd <workspace_dir>/src/dynamics_mpc_controller/launch && \
python3 dual_arm_joint_tracking_target.py
```

## Contact

- **Author**: Wei-Hsuan Cheng [(johnathancheng0125@gmail.com)](mailto:johnathancheng0125@gmail.com)
- **Homepage**: [wei-hsuan-cheng](https://wei-hsuan-cheng.github.io)
- **GitHub**: [wei-hsuan-cheng](https://github.com/wei-hsuan-cheng)
