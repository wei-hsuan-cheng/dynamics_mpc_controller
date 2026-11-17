#!/usr/bin/env python3
"""UR5 inverse-dynamics MPC demo: joint targets among moving obstacles, for tests/demo_video/ur5_demo_video.sh.

With the controller running (ur5.launch.py, inverse_dynamics_mpc_controller), waits for the MPC to publish
/mpc_observation, then starts the two example publishers with their default motion, only use_sim_time set:
  example/command/mpc_targets/joint_tracking_target.py  joint sine about the initial pose
  example/command/obstacles/moving_spheres.py           three moving obstacles on /dynamic_obstacles
runs them for DURATION seconds of simulation time, stops them, and writes PREFIX_labels.json: the time window
(start, end), sim seconds from the start of both publishers, which is what the videos show.

  python3 tests/demo_video/run_ur5_demo.py --prefix /tmp/ur5_demo/run [--duration 30]
"""
import argparse
import json
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.parameter import Parameter
from ocs2_msgs.msg import MpcObservation

ROOT = Path(__file__).resolve().parents[2]
COMMANDS = [
    ROOT / "example/command/mpc_targets/joint_tracking_target.py",
    ROOT / "example/command/obstacles/moving_spheres.py",
]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--prefix", required=True)
    parser.add_argument("--duration", type=float, default=30.0, help="sim seconds of commanded motion")
    parser.add_argument("--timeout", type=float, default=300.0, help="wall seconds to wait for the MPC")
    args = parser.parse_args()
    prefix = Path(args.prefix)
    prefix.parent.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = rclpy.create_node("ur5_demo", parameter_overrides=[Parameter("use_sim_time", value=True)])
    observations = []
    node.create_subscription(MpcObservation, "/mpc_observation", lambda m: observations.append(m.time), 10)

    def now():
        return node.get_clock().now().nanoseconds * 1e-9

    # The controller publishes /mpc_observation only once it is active, after its initial solve.
    deadline = time.time() + args.timeout
    while len(observations) < 50 and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if len(observations) < 50:
        print("no /mpc_observation: the MPC controller did not start", file=sys.stderr)
        return 1
    settle = now() + 1.0
    while now() < settle:
        rclpy.spin_once(node, timeout_sec=0.05)

    start = now()
    processes = [subprocess.Popen([sys.executable, str(command), "--ros-args", "-p", "use_sim_time:=true"])
                 for command in COMMANDS]
    print(f"commands started at sim t = {start:.3f} s", flush=True)
    end = start + args.duration
    while now() < end + 0.5:
        rclpy.spin_once(node, timeout_sec=0.05)
        if any(p.poll() is not None for p in processes):
            print("a command publisher exited early", file=sys.stderr)
            break
    for p in processes:
        p.send_signal(signal.SIGINT)
    for p in processes:
        try:
            p.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            p.kill()
    Path(f"{prefix}_labels.json").write_text(json.dumps(dict(start=start, end=end)) + "\n")
    print(f"wrote {prefix}_labels.json: sim t = {start:.3f} .. {end:.3f} s", flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
