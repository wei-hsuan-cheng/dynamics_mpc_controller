#!/usr/bin/env python3
"""Three moving obstacles for the UR5 inverse-dynamics MPC's dynamicObstacles.

Stands in for a vision tracker: publishes dynamics_mpc_controller/TrackedObstacleArray (pose, velocity and
sphere shape of every obstacle, the complete set each message) and the same spheres as RViz markers.
The red wall and the blue box are not here: they stay fixed selfCollision pairs.

  python3 src/dynamics_mpc_controller/example/command/obstacles/moving_spheres.py --ros-args -p use_sim_time:=true

  obstacle 1  sphere r=0.06, circling the arm at shoulder height
  obstacle 2  sphere r=0.07, bobbing up and down in front of the arm
  obstacle 3  two spheres r=0.05 (a dumbbell), sweeping sideways while spinning about z

Parameters: topic (/dynamic_obstacles), marker_topic (/dynamic_obstacles/markers), frame_id (world),
rate (30 Hz), speed (1.0, scales every motion), blink_period (0: off; > 0: obstacle 2 disappears for
half of every period, to show obstacles coming and going), position_std (0.0 m).
"""

import math

import rclpy
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from dynamics_mpc_controller.msg import TrackedObstacle, TrackedObstacleArray
from visualization_msgs.msg import Marker, MarkerArray


def obstacle(identifier, position, velocity, yaw, yaw_rate, centers, radii, position_std):
    o = TrackedObstacle()
    o.id = identifier
    o.pose.position.x, o.pose.position.y, o.pose.position.z = position
    o.pose.orientation.z = math.sin(0.5 * yaw)
    o.pose.orientation.w = math.cos(0.5 * yaw)
    o.twist.linear.x, o.twist.linear.y, o.twist.linear.z = velocity
    o.twist.angular.z = yaw_rate
    o.sphere_centers = [Point(x=c[0], y=c[1], z=c[2]) for c in centers]
    o.sphere_radii = list(radii)
    o.position_std = position_std
    return o


class MovingSpheres(Node):
    def __init__(self):
        super().__init__("moving_spheres")
        self.frame_id = self.declare_parameter("frame_id", "world").value
        self.speed = float(self.declare_parameter("speed", 1.0).value)
        self.blink_period = float(self.declare_parameter("blink_period", 0.0).value)
        self.position_std = float(self.declare_parameter("position_std", 0.0).value)
        topic = self.declare_parameter("topic", "/dynamic_obstacles").value
        marker_topic = self.declare_parameter("marker_topic", "/dynamic_obstacles/markers").value
        rate = float(self.declare_parameter("rate", 30.0).value)
        self.publisher = self.create_publisher(TrackedObstacleArray, topic, 1)
        self.markers = self.create_publisher(MarkerArray, marker_topic, 1)
        self.start = None
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(f"publishing 3 moving obstacles on {topic} in frame '{self.frame_id}'")

    def obstacles(self, t):
        s = self.speed
        std = self.position_std
        out = []

        # 1: circle of radius 0.55 m around the base, z = 0.6 m (below the blue box).
        w1 = 0.4 * s
        a = w1 * t
        out.append(obstacle(
            1, (0.55 * math.cos(a), 0.55 * math.sin(a), 0.6),
            (-0.55 * w1 * math.sin(a), 0.55 * w1 * math.cos(a), 0.0),
            0.0, 0.0, [(0.0, 0.0, 0.0)], [0.06], std))

        # 2: up and down in front of the arm, z in [0.3, 0.6] m.
        w2 = 0.6 * s
        visible = self.blink_period <= 0.0 or (t % self.blink_period) < 0.5 * self.blink_period
        if visible:
            out.append(obstacle(
                2, (0.45, -0.25, 0.45 + 0.15 * math.sin(w2 * t)),
                (0.0, 0.0, 0.15 * w2 * math.cos(w2 * t)),
                0.0, 0.0, [(0.0, 0.0, 0.0)], [0.07], std))

        # 3: two spheres 0.16 m apart, sweeping along y at x = 0.55 m, z = 0.45 m, spinning about z.
        w3 = 0.3 * s
        spin = 1.0 * s
        out.append(obstacle(
            3, (0.55, 0.35 * math.sin(w3 * t), 0.45),
            (0.0, 0.35 * w3 * math.cos(w3 * t), 0.0),
            spin * t, spin, [(0.08, 0.0, 0.0), (-0.08, 0.0, 0.0)], [0.05, 0.05], std))
        return out

    def tick(self):
        now = self.get_clock().now()
        if self.start is None:
            self.start = now
        t = (now - self.start).nanoseconds * 1e-9
        msg = TrackedObstacleArray()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id
        msg.obstacles = self.obstacles(t)
        self.publisher.publish(msg)
        self.markers.publish(self.marker_array(msg))

    def marker_array(self, msg):
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        colors = {1: (1.0, 0.6, 0.0), 2: (0.0, 0.8, 0.3), 3: (0.8, 0.2, 0.9)}
        for o in msg.obstacles:
            q = o.pose.orientation
            yaw = 2.0 * math.atan2(q.z, q.w)
            for j, (c, r) in enumerate(zip(o.sphere_centers, o.sphere_radii)):
                m = Marker()
                m.header = msg.header
                m.ns = "dynamic_obstacles"
                m.id = 10 * o.id + j
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                m.pose.position.x = o.pose.position.x + math.cos(yaw) * c.x - math.sin(yaw) * c.y
                m.pose.position.y = o.pose.position.y + math.sin(yaw) * c.x + math.cos(yaw) * c.y
                m.pose.position.z = o.pose.position.z + c.z
                m.pose.orientation.w = 1.0
                m.scale.x = m.scale.y = m.scale.z = 2.0 * r
                m.color.r, m.color.g, m.color.b = colors.get(o.id, (0.5, 0.5, 0.5))
                m.color.a = 0.8
                markers.markers.append(m)
        return markers


def main():
    rclpy.init()
    node = MovingSpheres()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
