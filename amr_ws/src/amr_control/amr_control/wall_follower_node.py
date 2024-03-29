import rclpy
from rclpy.node import Node

import message_filters
from amr_msgs.msg import PoseStamped, RangeScan
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

from typing import List

from amr_control.wall_follower import WallFollower


class WallFollowerNode(Node):
    def __init__(self):
        """Wall follower node initializer."""
        super().__init__("wall_follower")

        # Parameters
        self.declare_parameter("dt", 0.05)
        dt = self.get_parameter("dt").get_parameter_value().double_value

        self.declare_parameter("enable_localization", False)
        enable_localization = (
            self.get_parameter("enable_localization").get_parameter_value().bool_value
        )

        # TODO: 1.7. Subscribe to /odom and /us_scan and sync them with _compute_commands_callback.
        self._subscribers: list[message_filters.Subscriber] = []
        self._subscribers.append(message_filters.Subscriber(self, Odometry, "odom"))
        self._subscribers.append(message_filters.Subscriber(self, RangeScan, "us_scan"))
        # TODO: could this be the problem????
        self._subscribers.append(message_filters.Subscriber(self, PoseStamped, "pose"))

        ts = message_filters.ApproximateTimeSynchronizer(self._subscribers, queue_size=10, slop=10)
        ts.registerCallback(self._compute_commands_callback)

        # TODO: 1.10. Create the /cmd_vel velocity commands publisher (TwistStamped message).
        self._publisher_vel = self.create_publisher(
            msg_type=TwistStamped, topic="cmd_vel", qos_profile=10
        )

        # Attribute and object initializations
        self._wall_follower = WallFollower(dt)

    def _compute_commands_callback(
        self, odom_msg: Odometry, us_msg: RangeScan, pose_msg: PoseStamped = PoseStamped()
    ):
        """Subscriber callback. Executes a wall-following controller and publishes v and w commands.

        Ceases to operate once the robot is localized.

        Args:
            odom_msg: Message containing odometry measurements.
            us_msg: Message containing US sensor readings.
            pose_msg: Message containing the estimated robot pose.

        """
        if not pose_msg.localized:
            # TODO: 1.8. Parse the odometry from the Odometry message (i.e., read z_v and z_w).
            z_v: float = 0.0
            z_w: float = 0.0

            z_v = odom_msg.twist.twist.linear.x  # CHECK
            z_w = odom_msg.twist.twist.angular.z

            # TODO: 1.9. Parse US measurements from the RangeScan message (i.e., read z_us).
            z_us: List[float] = []
            z_us = us_msg.ranges

            # Execute wall follower
            v, w = self._wall_follower.compute_commands(z_us, z_v, z_w)
            self.get_logger().info(f"Commands: v = {v:.3f} m/s, w = {w:+.3f} rad/s")

            # Publish
            self._publish_velocity_commands(v, w)

    def _publish_velocity_commands(self, v: float, w: float) -> None:
        """Publishes velocity commands in a geometry_msgs.msg.TwistStamped message.

        Args:
            v: Linear velocity command [m/s].
            w: Angular velocity command [rad/s].

        """

        # TODO: 1.11. Complete the function body with your code (i.e., replace the pass statement).
        msg = TwistStamped()
        msg.twist.linear.x = v
        msg.twist.angular.z = w
        self._publisher_vel.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    wall_follower_node = WallFollowerNode()

    try:
        rclpy.spin(wall_follower_node)
    except KeyboardInterrupt:
        pass

    wall_follower_node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
