# Dynamics MPC Controller

ROS 2 controller package for fixed-base inverse/forward dynamics MPC using [OCS2](https://github.com/wei-hsuan-cheng/ocs2_ros2.git) and [Pinocchio](https://github.com/stack-of-tasks/pinocchio.git).

- The inverse dynamics MPC formulation uses [`pinocchio::rnea()`](./src/inverse_dynamics_mpc/constraint/inverse_dynamics_rnea_with_ee_wrench_constraint_cppad.cpp#L59) as state-input equality constraint.
- The forward dynamics MPC formulation uses [`pinocchio::aba()`](./src/forward_dynamics_mpc/dynamics/forward_dynamics_aba_dynamics_ad.cpp#L41) as system dynamics model.

This is the **ROS 2 Jazzy** branch (Ubuntu 24.04, ros2_control 4.x, Pinocchio 4). The Humble version is on `humble`.

Full details about the dynamics mpc formulation (model/cost/constraint/solver/etc.) can be found in [`dynamics_mpc_formulation.md`](./docs/dynamics_mpc_formulation.md).


## Build and Install

- Clone this repo
  ```bash
  git clone \
    https://github.com/wei-hsuan-cheng/dynamics_mpc_controller.git \
    -b jazzy
  ```

- Clone all sub-repo with vcs
  ```bash
  cd <workspace_dir>/src
  mkdir dynamics_mpc_controller_dependencies
  vcs import < dynamics_mpc_controller/dynamics_mpc_controller.repos
  ```

- First install by `rosdep`
  ```bash
  # rosdep install
  cd <workspace_dir>
  sudo rosdep init # if you never did this
  rosdep update
  rosdep install --ignore-src --from-paths src -y -r
  ```

- Build pkgs up-to `dynamics_mpc_controller`
  ```bash
  cd <workspace_dir>
  NUM_JOBS=2 && \
  export CMAKE_BUILD_PARALLEL_LEVEL=${NUM_JOBS} && \
  export MAKEFLAGS=-j${NUM_JOBS} && \
  export NINJAFLAGS=-j${NUM_JOBS} && \
  colcon build --symlink-install \
    --packages-up-to dynamics_mpc_controller \
    --executor sequential --parallel-workers ${NUM_JOBS} \
    --cmake-force-configure \
    --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release && \
    . install/setup.bash
  ```


## Run MuJoCo Example

Single `ur5` example:
```bash
ros2 launch dynamics_mpc_controller ur5.launch.py \
  mpcControllerName:=inverse_dynamics_mpc_controller \
  mujoco_headless:=false \
  mpcFreq:=100
```

Dual `ur5` example:
```bash
ros2 launch dynamics_mpc_controller dual_ur5.launch.py \
  mpcControllerName:=inverse_dynamics_mpc_controller \
  mujoco_headless:=false \
  mpcFreq:=100
```

Useful launch args:
```bash
mpcControllerName:=inverse_dynamics_mpc_controller | forward_dynamics_mpc_controller
mujoco_headless:=false | true
mujoco_real_time_factor:=1.0 # double
mujoco_publish_rate:=100.0 # double
mpcFreq:=100 # should be integer
mrtFreq:=1000 # should be integer
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
cd <workspace_dir>
# Joint target
python3 src/dynamics_mpc_controller/example/command/mpc_targets/joint_tracking_target.py
# Cartesian
python3 src/dynamics_mpc_controller/example/command/mpc_targets/ee_motion_tracking_target.py
```

Dual `ur5` sample code:
```bash
cd <workspace_dir>
python3 src/dynamics_mpc_controller/example/command/mpc_targets/dual_arm_joint_tracking_target.py
```


## Contact

- **Author**: Wei-Hsuan Cheng [(johnathancheng0125@gmail.com)](mailto:johnathancheng0125@gmail.com)
- **Homepage**: [wei-hsuan-cheng](https://wei-hsuan-cheng.github.io)
- **GitHub**: [wei-hsuan-cheng](https://github.com/wei-hsuan-cheng)
