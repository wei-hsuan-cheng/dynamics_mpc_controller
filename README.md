# Dynamics MPC Controller

ROS 2 controller package for fixed-base inverse/forward dynamics MPC using [OCS2](https://github.com/wei-hsuan-cheng/ocs2_ros2.git) and [Pinocchio](https://github.com/stack-of-tasks/pinocchio.git).

- The inverse dynamics MPC formulation uses [`pinocchio::rnea()`](./src/inverse_dynamics_mpc/constraint/inverse_dynamics_rnea_with_ee_wrench_constraint_cppad.cpp#L59) as state-input equality constraint.
- The forward dynamics MPC formulation uses [`pinocchio::aba()`](./src/forward_dynamics_mpc/dynamics/forward_dynamics_aba_dynamics_ad.cpp#L41) as system dynamics model.

This is the **ROS 2 Jazzy** branch (Ubuntu 24.04, ros2_control 4.x, Pinocchio 4). The Humble version is on `humble`.

Full details about the dynamics mpc formulation (model/cost/constraint/solver/etc.) can be found in [`dynamics_mpc_formulation.md`](./docs/dynamics_mpc_formulation.md).


## Demos

<p align="center">
  <img src="https://github.com/wei-hsuan-cheng/gifs/blob/main/dynamics_mpc_controller/ur5_moving_obstacles.gif?raw=true" width="800" alt="UR5 inverse-dynamics MPC tracking joint targets among moving obstacles" />
  <br />
  <b>UR5 inverse-dynamics MPC</b><br />Joint target tracking while avoiding tracked moving obstacles (left: RViz with the MPC plan, future ghosts and obstacle predictions; right: MuJoCo)
</p>


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

# Tracked moving obstacles (inverse dynamics MPC), and what the MPC sees of them
ros2 topic echo /dynamic_obstacles
ros2 topic echo /performance_visualization/dynamicObstacleMarkers
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


## Moving Obstacles

The inverse dynamics MPC keeps the arm away from tracked moving obstacles published on `/dynamic_obstacles` (`dynamics_mpc_controller/TrackedObstacleArray`), as a vision tracker would publish them: each obstacle is a set of spheres in its own frame, with a pose and a velocity.
The MPC extrapolates them at constant velocity over the horizon and keeps every collision primitive of the listed links `minimumDistance` away from every sphere with a relaxed barrier, on top of `selfCollision`'s fixed pairs (red wall, blue box).
The math is in [`dynamics_mpc_formulation.md`](./docs/dynamics_mpc_formulation.md#moving-obstacle-avoidance).

Single `ur5` sample code, with the controller running:
```bash
cd <workspace_dir>
python3 src/dynamics_mpc_controller/example/command/obstacles/moving_spheres.py --ros-args -p use_sim_time:=true
```

The example moves three obstacles around the arm: a sphere circling at shoulder height, one bobbing in front of the arm, and two spheres sweeping sideways while spinning.
`-p blink_period:=4.0` makes the second one come and go; `-p speed:=2.0` speeds them all up.
In RViz, the `Dynamic Obstacles` display (`/performance_visualization/dynamicObstacleMarkers`) shows what the MPC sees: the spheres now, their clearance shells, and their predicted paths and spheres at the robot-ghost times.
MuJoCo does not show them, and they do not collide physically.

`ocs2.task.dynamicObstacles` in [`config/ur5/ros2_controllers.yaml`](./config/ur5/ros2_controllers.yaml):

| key | meaning |
|---|---|
| `links` | arm links kept away from the spheres (their URDF collision primitives) |
| `minimumDistance`, `mu`, `delta` | clearance and relaxed barrier, the same as `selfCollision`'s |
| `maxObstacles`, `maxSpheresPerObstacle` | obstacle slots; beyond them the obstacles nearest the arm base are kept |
| `timeout` | no message for this long clears every obstacle |
| `predict`, `maxPredictionTime` | constant-velocity extrapolation of each obstacle over the horizon |
| `fadeInTime` | a new obstacle's barrier ramps in, so the plan does not jump |
| `positionStdScale` | radii inflated by this multiple of each obstacle's `position_std` |

Each message is the complete current set: a missing id is gone, and an id keeps its slot while it stays.

Measured in headless MuJoCo with both examples at their defaults (30 s), between the arm's collision primitives and the sphere surfaces (`true`: two runs, one of them the demo above):

| `dynamicObstacles.activate` | closest approach | time inside the 5 cm clearance | time penetrating | MPC update, average |
|---|---|---|---|---|
| `true` | 4 to 16 mm | 12 to 14 % | 0 % | 3.5 ms |
| `false` | -86 mm (penetrating) | 40 % | 18 % | 2.3 ms |

The clearance is a soft constraint: when the joint target pulls the arm towards an obstacle, the arm may enter the 5 cm band, but it has not touched a sphere.


## Demo Video

The demo above (both examples at their defaults for 30 s, RViz next to MuJoCo with the same camera) is reproduced in two steps; the run and the renders go to the git-ignored `demo_videos/`:
```bash
# in the container, with an X display for RViz (DISPLAY, default :1); about 5 min
cd <workspace_dir>/src/dynamics_mpc_controller
tests/demo_video/ur5_demo_video.sh record

# where MuJoCo 3.13, numpy, Pillow and ffmpeg are installed
PYTHON=<python with mujoco> tests/demo_video/ur5_demo_video.sh render
# -> demo_videos/ur5_obstacles/ur5_obstacles.mp4 (labels) and ur5_obstacles_clean.mp4
```


## Contact

- **Author**: Wei-Hsuan Cheng [(johnathancheng0125@gmail.com)](mailto:johnathancheng0125@gmail.com)
- **Homepage**: [wei-hsuan-cheng](https://wei-hsuan-cheng.github.io)
- **GitHub**: [wei-hsuan-cheng](https://github.com/wei-hsuan-cheng)
