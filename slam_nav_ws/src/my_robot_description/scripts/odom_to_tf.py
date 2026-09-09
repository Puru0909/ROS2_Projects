#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class OdomToTF(Node):

    def __init__(self):
        super().__init__('odom_to_tf_broadcaster')

        self.br = TransformBroadcaster(self)

        self.sub = self.create_subscription(
            Odometry,
            '/odom',
            self.on_odom,
            50
        )

    def on_odom(self, msg: Odometry):

        t = TransformStamped()

        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = 0.0

        t.transform.rotation = msg.pose.pose.orientation

        self.br.sendTransform(t)


def main(args=None):

    rclpy.init(args=args)

    node = OdomToTF()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()