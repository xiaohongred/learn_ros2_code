#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class PoseSubNode(Node):

    def __init__(self):
        super().__init__("pose_suber")
        self.counter_ = 0
        self.pose_sub = self.create_subscription(Pose, "/turtle1/pose", self.subCallback, 10)
        self.get_logger().info("pose subscriber node started")
        

    def subCallback(self, msg: Pose):
        self.get_logger().info(str(msg))
        

def main(args=None):
    rclpy.init(args=args)

    node = PoseSubNode()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()