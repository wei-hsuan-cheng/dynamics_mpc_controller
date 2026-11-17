#!/bin/bash
# UR5 demo video: the inverse-dynamics MPC tracking the example joint targets among the example moving obstacles, as
# one side-by-side video, RViz on the left (MPC plan, future ghosts, obstacles with their clearance and prediction)
# and MuJoCo on the right, same camera and field of view, matched by sim time.
#
#   record [NAME]  in the ros_jazzy container, with an X display for RViz (DISPLAY, default :1):
#                  1. runs ur5.launch.py headless with the inverse-dynamics MPC, starts joint_tracking_target.py and
#                     moving_spheres.py (default motion, use_sim_time only) for 30 s of sim time (DURATION), and
#                     records a bag of what RViz shows plus the joint states and the obstacles;
#                  2. steps RViz through the bag one video frame at a time and screenshots its render area
#                     (about 3 min; RViz's window shows on that display meanwhile: keep it uncovered).
#   render [NAME]  where MuJoCo 3.13, numpy, Pillow and ffmpeg are installed (PYTHON, default python3): replays the run
#                  in MuJoCo with the same camera next to the RViz frames: NAME.mp4 (labels), NAME_clean.mp4.
#
# Everything goes to <repo>/demo_videos/NAME/ (git-ignored, default NAME ur5_obstacles): run_* (labels, replay,
# obstacles), launch.log, bag/, rviz/ (frames), the videos. ~/src is mounted at /root/src in the container, so the
# repository is the same folder on the host: record in the container and render on the host without copying anything.
CMD=${1:?usage: ur5_demo_video.sh record|render [NAME]}
NAME=${2:-ur5_obstacles}
R=$(cd "$(dirname "$0")/../.." && pwd)
D=$R/demo_videos/$NAME
mkdir -p "$D"
case $CMD in
  record)
    source /root/ocs2_ros2_ws/install/setup.bash
    cd /root/ocs2_ros2_ws  # the CppAD library folder auto_generated/ur5 is relative to the workspace
    rm -rf "$D/bag" "$D/rviz"
    ros2 launch dynamics_mpc_controller ur5.launch.py mpcControllerName:=inverse_dynamics_mpc_controller \
      mujoco_headless:=true rviz:=false > "$D/launch.log" 2>&1 &
    LAUNCH=$!
    ros2 bag record -o "$D/bag" --use-sim-time \
      -e '/performance_visualization/.*|/tf|/tf_static|/robot_description|/clock|/joint_states|/dynamic_obstacles' \
      > "$D/bag.log" 2>&1 &
    BAG=$!
    python3 "$R/tests/demo_video/run_ur5_demo.py" --prefix "$D/run" --duration "${DURATION:-30}"
    STATUS=$?
    sleep 1
    kill -TERM $BAG $LAUNCH  # background jobs ignore SIGINT
    wait $BAG $LAUNCH
    [ $STATUS -eq 0 ] && [ -f "$D/run_labels.json" ] || { echo "the demo did not run (see $D/launch.log)"; exit 1; }
    python3 "$R/tests/demo_video/extract_ur5_replay.py" "$D/bag" "$D/run" || exit 1
    python3 "$R/tests/demo_video/render_rviz_replay.py" "$D/bag" "$D/run_labels.json" "$D/rviz" \
      --joints "$D/run_replay.json" --display "${DISPLAY:-:1}" || exit 1
    echo "recorded $D; now: tests/demo_video/ur5_demo_video.sh render $NAME"
    ;;
  render)
    PY=${PYTHON:-python3}
    "$PY" "$R/tests/demo_video/render_ur5_video.py" "$D/run" --rviz-frames "$D/rviz" --out "$D/$NAME.mp4" &
    "$PY" "$R/tests/demo_video/render_ur5_video.py" "$D/run" --rviz-frames "$D/rviz" --no-text --out "$D/${NAME}_clean.mp4"
    wait
    ls -la "$D"/*.mp4
    ;;
  *)
    echo "usage: ur5_demo_video.sh record|render [NAME]"
    exit 2
    ;;
esac
