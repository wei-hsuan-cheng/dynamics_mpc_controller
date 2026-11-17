#!/usr/bin/env python3
"""Joint angles and tracked obstacles of a recorded UR5 run, for the RViz and MuJoCo replays of the demo video.

usage: extract_ur5_replay.py BAG PREFIX [--rate 200]

Reads /joint_states and /dynamic_obstacles from BAG (recorded with --use-sim-time) and writes, timed by the message
header stamps (simulation time):
  PREFIX_replay.json     [{time, names (first sample only), q}], joint angles at up to RATE Hz
  PREFIX_obstacles.json  [{time, obstacles: [{id, position, quaternion_wxyz, linear, angular, centers, radii}]}],
                         every obstacle message
Run in the sourced workspace (rosbag2_py; the obstacle message type comes from dynamics_mpc_controller).
"""
import argparse
import json
from pathlib import Path

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bag", type=Path)
    parser.add_argument("prefix")
    parser.add_argument("--rate", type=float, default=200.0, help="joint samples per second at most")
    args = parser.parse_args()

    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(args.bag), storage_id=""), ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    joints, obstacles = [], []
    names = None
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic == "/joint_states":
            message = deserialize_message(data, get_message(types[topic]))
            # The header stamp (simulation time of the sample): the bag time only advances with /clock.
            t = message.header.stamp.sec + 1e-9 * message.header.stamp.nanosec or stamp * 1e-9
            if joints and t - joints[-1]["time"] < 1.0 / args.rate:
                continue
            sample = dict(time=t, q=list(message.position))
            if names is None:
                names = list(message.name)
                sample["names"] = names
            elif list(message.name) != names:
                order = [list(message.name).index(n) for n in names]
                sample["q"] = [message.position[i] for i in order]
            joints.append(sample)
        elif topic == "/dynamic_obstacles":
            message = deserialize_message(data, get_message(types[topic]))
            t = message.header.stamp.sec + 1e-9 * message.header.stamp.nanosec
            obstacles.append(dict(time=t if t > 0.0 else stamp * 1e-9, obstacles=[dict(
                id=o.id,
                position=[o.pose.position.x, o.pose.position.y, o.pose.position.z],
                quaternion_wxyz=[o.pose.orientation.w, o.pose.orientation.x, o.pose.orientation.y, o.pose.orientation.z],
                linear=[o.twist.linear.x, o.twist.linear.y, o.twist.linear.z],
                angular=[o.twist.angular.x, o.twist.angular.y, o.twist.angular.z],
                centers=[[c.x, c.y, c.z] for c in o.sphere_centers],
                radii=list(o.sphere_radii)) for o in message.obstacles]))
    if not joints:
        raise SystemExit("no /joint_states in the bag")
    Path(f"{args.prefix}_replay.json").write_text(json.dumps(joints))
    Path(f"{args.prefix}_obstacles.json").write_text(json.dumps(obstacles))
    print(f"wrote {len(joints)} joint samples and {len(obstacles)} obstacle messages to {args.prefix}_*.json")


if __name__ == "__main__":
    main()
