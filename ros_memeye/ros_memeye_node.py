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
from transformers import TrOCRProcessor, VisionEncoderDecoderModel, DetrImageProcessor, DetrForObjectDetection
import requests
from PIL import Image as PIL_Image
from paddleocr import PaddleOCR,draw_ocr
from numpy import asarray
import torch


class RosMemeyeNode(Node):
    def __init__(self):
        super().__init__("ros_memeye_node")
        # read the openAI api key from system variable
        self.api_key = os.getenv("OPENAI_API_KEY")
        self.ocr_publisher = self.create_publisher(String, 'ocr_result', 10)
        self.detr_publisher = self.create_publisher(String, 'detr_result', 10)
        self.img_subscription = self.create_subscription(
            Image,
            'user_img',
            self.img_callback,
            10)
        self.bridge = CvBridge()

        self.ocr_recognizer = PaddleOCR(use_angle_cls=True, lang='en')
        self.get_logger().info("============OCR Model Ready===========")
        
        self.detr_processor = DetrImageProcessor.from_pretrained("facebook/detr-resnet-50")
        self.detr_model = DetrForObjectDetection.from_pretrained("facebook/detr-resnet-50")
        self.get_logger().info("============Detection Model Ready===========")
        
        
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
        
        inputs = self.detr_processor(images=image, return_tensors="pt")
        outputs = self.detr_model(**inputs)
        target_sizes = torch.tensor([image.size[::-1]])
        results = self.detr_processor.post_process_object_detection(outputs, target_sizes=target_sizes, threshold=0.9)[0]
        
        detr_result = ""
        for score, label, box in zip(results["scores"], results["labels"], results["boxes"]):
            box = [round(i, 2) for i in box.tolist()]
            detr_result += \
                f"Detected {self.detr_model.config.id2label[label.item()]} with confidence " + \
                f"{round(score.item(), 3)} at location {box}\n"
        
        detr_msg = String()
        detr_msg.data = detr_result
        self.detr_publisher.publish(detr_msg)



def main(args=None):
    rclpy.init(args=args)
    rosMemeyeNode = RosMemeyeNode()
    rclpy.spin(rosMemeyeNode)
    rosMemeyeNode.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()