#!usr/bin/env/python3

import rclpy
from rclpy import Node
from turtlesim.msg import Pose


#node 
class PoseSubscriberNode(Node):
    
    def __init__(self):
        super().__init__("pose_subscriber")
        self.pose_subscriber_ = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10) # subscriber
        
    def pose_callback(self, msg: Pose):
        self.get_logger().info("(" + str(msg.x) + "," + str(msg.y) + ")")
        

# main function
def main(args=Node):
    rclpy.init(args=args)
    node = PoseSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()