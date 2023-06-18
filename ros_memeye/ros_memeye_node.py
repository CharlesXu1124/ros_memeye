#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import openai
import numpy as np
import os
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge



class RosMemeyeNode(Node):
    def __init__(self):
        super().__init__("ros_memeye_node")
        # read the openAI api key from system variable
        self.api_key = os.getenv("OPENAI_API_KEY")
        self.subscription = self.create_subscription(
            Image,
            '/user_img',
            self.img_callback,
            10)
        self.bridge = CvBridge()
        self.subscription
        
    def img_callback(self, Image):
        self.get_logger().info("image received")
        cv_image = self.bridge.imgmsg_to_cv2(Image, desired_encoding='passthrough')


def main(args=None):
    rclpy.init(args=args)
    rosMemeyeNode = RosMemeyeNode()
    rclpy.spin(rosMemeyeNode)
    rosMemeyeNode.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()