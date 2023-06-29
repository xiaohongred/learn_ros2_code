#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DrawCircleNode(Node):

    def __init__(self):
        super().__init__("draw_circle")
        self.counter_ = 0
        self.turtle_cmd_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.timer_ = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("Draw circle node has been started")
        

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 1.0
        self.turtle_cmd_pub.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)

    node = DrawCircleNode()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()