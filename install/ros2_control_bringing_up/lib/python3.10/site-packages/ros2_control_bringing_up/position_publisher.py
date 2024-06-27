#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import random

class PubNode(Node):
    def __init__(self):
        super().__init__("position_publisher")
        self.publisher = self.create_publisher(Pose, "position", 10)
        self.timer_ = self.create_timer(0.5, self.publish_position)

    def publish_position(self):
        msg = Pose()
        msg.position.x = random.uniform(0.0, 2.0)
        msg.position.y = random.uniform(0.0, 2.0)
        msg.position.z = random.uniform(0.0, 2.0)
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 1.0
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PubNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()