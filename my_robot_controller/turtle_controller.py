#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen
from functools import partial

class TurtleControllerNode(Node):

    def __init__(self):
        super().__init__("turtle_controller")
        self.turtle_cmd_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.pose_sub = self.create_subscription(Pose, "/turtle1/pose", self.subCallback, 10)
        self.pre_x = 0
        self.get_logger().info("pose subscriber node started")


    # pose_sub call back func
    def subCallback(self, msg: Pose):
        cmd = Twist()
        cmd.linear.x = 5.0
        cmd.angular.z = 0.0
        if msg.x > 8.0 or msg.x < 2.0 or msg.y > 9.0 or msg.y < 2.0:
            cmd.linear.x = 1.0
            cmd.angular.z = 1.0
            
        else:
            cmd.linear.x = 4.0
            cmd.angular.z = 0.0
        
        self.turtle_cmd_pub.publish(cmd)
        if msg.x > 6.5 and self.pre_x <= 6.5:
            self.call_set_pen_service(255, 0, 0, 3, 0)
            self.pre_x = msg.x
        elif msg.x <2.0 and self.pre_x > 2.0:
            self.pre_x = msg.x
            self.call_set_pen_service(0, 0, 255, 3, 0)

    def call_set_pen_service(self, r, g, b, width, off):
        client = self.create_client(SetPen, "/turtle1/set_pen")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service ...")
        penCmd = SetPen.Request()
        penCmd.r = r
        penCmd.g = g
        penCmd.b = b
        penCmd.width = width
        penCmd.off = off
        future = client.call_async(penCmd)
        future.add_done_callback(partial(self.callback_set_pen))

    def callback_set_pen(self, future):
        try:
            resp = future.result()
            self.get_logger().info("Set Pen color success %r"%resp)
        except Exception as e:
            self.get_logger().error("Service call failed: %r"%(e,))
        

def main(args=None):
    rclpy.init(args=args)

    node = TurtleControllerNode()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()