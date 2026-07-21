# Dynamics MPC Controller

ROS 2 controller package for fixed-base inverse/forward dynamics MPC using [OCS2](https://github.com/wei-hsuan-cheng/ocs2_ros2.git) and [Pinocchio](https://github.com/stack-of-tasks/pinocchio.git).

- The inverse dynamics MPC formulation uses [`pinocchio::rnea()`](./src/inverse_dynamics_mpc/constraint/inverse_dynamics_rnea_with_ee_wrench_constraint_cppad.cpp#L59) as state-input equality constraint.
- The forward dynamics MPC formulation uses [`pinocchio::aba()`](./src/forward_dynamics_mpc/dynamics/forward_dynamics_aba_dynamics_ad.cpp#L41) as system dynamics model.

Full details about the dynamics mpc formulation (model/cost/constraint/solver/etc.) can be found in [`dynamics_mpc_formulation.md`](./docs/dynamics_mpc_formulation.md).


## Build and Install

- Clone this repo
  ```bash
  git clone \
    https://github.com/wei-hsuan-cheng/dynamics_mpc_controller.git \
    -b main
  ```

- Clone all sub-repo with vcs
  ```bash
  cd <workspace_dir>/src
  mkdir dynamics_mpc_controller_dependencies
  vcs import < dynamics_mpc_controller/dynamics_mpc_controller.repos
  ```

- Install `pinocchio` library (**3.9.x required**; `packages.ros.org` only serves the newest
  build, which is `4.0.0` now, so install `3.9.0` from the ROS snapshot archive and hold it)
    ```bash
    # Import the ROS snapshot archive key and add the 2026-03-29 humble snapshot
    # (works on x86_64 and arm64; the arch is taken from dpkg)
    curl -s "https://keyserver.ubuntu.com/pks/lookup?op=get&search=0xAD19BAB3CBF125EA" | \
      sudo gpg --batch --yes --dearmor -o /usr/share/keyrings/ros-snapshots-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-snapshots-archive-keyring.gpg] http://snapshots.ros.org/humble/2026-03-29/ubuntu jammy main" | \
      sudo tee /etc/apt/sources.list.d/ros2-snapshots.list
    sudo apt update

    # Install pinocchio 3.9.0 from the snapshot and pin it so a later apt upgrade
    # does not pull 4.x. The exact build id differs per architecture
    # (amd64: ...20260304.203533, arm64: ...20260307.163259), so resolve it from apt:
    PINOCCHIO_VERSION=$(apt-cache madison ros-humble-pinocchio | awk '/snapshots.ros.org/ {print $3; exit}')
    sudo apt install ros-humble-pinocchio=${PINOCCHIO_VERSION}
    sudo apt-mark hold ros-humble-pinocchio

    # Drop the snapshot source again afterwards
    sudo rm /etc/apt/sources.list.d/ros2-snapshots.list && sudo apt update

    # Check the installed version and the hold:
    # expect "Version: 3.9.0-..." and flag "hi" (h = held, i = installed)
    dpkg -s ros-humble-pinocchio | grep Version
    dpkg -l ros-humble-pinocchio | tail -1
    ```

- First install by `rosdep`
  ```bash
  # rosdep install
  cd <workspace_dir>
  sudo rosdep init # if you never did this
  rosdep update
  rosdep install --ignore-src --from-paths src -y -r
  ```

- Build `mujoco`-related pkg. (See **troubleshooting section** [here](https://github.com/wei-hsuan-cheng/mujoco_ros2_control) if needed)
  ```bash
  cd <workspace_dir>
  NUM_JOBS=2 && \
  export CMAKE_BUILD_PARALLEL_LEVEL=${NUM_JOBS} && \
  export MAKEFLAGS=-j${NUM_JOBS} && \
  export NINJAFLAGS=-j${NUM_JOBS} && \
  colcon build --symlink-install \
    --packages-up-to mujoco_ros2_control mujoco_ros2_control_demos \
    --executor sequential --parallel-workers ${NUM_JOBS} \
    --cmake-force-configure \
    --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release && \
    . install/setup.bash
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
mujoco_real_time_factor:=1.0 # double
mujoco_publish_rate:=100.0 # double
mpcFreq:=50 # should be integer
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
