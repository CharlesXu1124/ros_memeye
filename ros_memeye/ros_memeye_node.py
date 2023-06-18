#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import openai
import numpy as np
import os
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from transformers import TrOCRProcessor, VisionEncoderDecoderModel
import requests
from PIL import Image as PIL_Image
from paddleocr import PaddleOCR,draw_ocr
from numpy import asarray


class RosMemeyeNode(Node):
    def __init__(self):
        super().__init__("ros_memeye_node")
        # read the openAI api key from system variable
        self.api_key = os.getenv("OPENAI_API_KEY")
        self.ocr_publisher = self.create_publisher(String, 'ocr_result', 10)
        self.img_subscription = self.create_subscription(
            Image,
            'user_img',
            self.img_callback,
            10)
        self.bridge = CvBridge()
        
        # self.processor = TrOCRProcessor.from_pretrained("microsoft/trocr-base-handwritten")
        # self.model = VisionEncoderDecoderModel.from_pretrained("microsoft/trocr-base-handwritten")
        self.ocr_recognizer = PaddleOCR(use_angle_cls=True, lang='en')
        self.get_logger().info("============OCR Model Ready===========")
        # prevent variable not used warning
        self.img_subscription

    def img_callback(self, Image):
        self.get_logger().info("image received")
        cv_image = self.bridge.imgmsg_to_cv2(Image, desired_encoding='passthrough')
        image = PIL_Image.fromarray(cv_image).convert("RGB")
        img_data = asarray(image)
        result = self.ocr_recognizer.ocr(img_data, cls=True)
        ocr_result = ""
        for idx in range(len(result)):
            res = result[idx]
            for line in res:
                ocr_result += line[1][0]
                ocr_result += "\n"
                
        ocr_msg = String()
        ocr_msg.data = ocr_result
        self.ocr_publisher.publish(ocr_msg)



def main(args=None):
    rclpy.init(args=args)
    rosMemeyeNode = RosMemeyeNode()
    rclpy.spin(rosMemeyeNode)
    rosMemeyeNode.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()