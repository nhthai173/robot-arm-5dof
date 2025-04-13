import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from .Detect import *
import numpy as np
import os
import json

os.environ['ROS_DOMAIN_ID'] = '173'
os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
os.environ['ROS_LOCALHOST_ONLY'] = '0'


class ImageSubscriber(Node):
    def __init__(self, name):
        super().__init__(name)
        self.sub = self.create_subscription(Image, 'image', self.listener_callback, 1)
        self.publisher_ = self.create_publisher(String, 'control', 1)
        self.cv_bridge = CvBridge()
        self.model = YOLO('best.pt')
        
        # Màu cho mask của các đối tượng
        self.colors = {
            'Goc': (35, 73, 42),
            'Den_doc': (0, 0, 0),
            'Den_ngang': (80, 80, 80),
            'Cam': (134, 176, 102),
            'Do': (54, 71, 188),
            'Vang': (255, 255, 255),
        }

    def yolo_detect(self, image):
        original_height, original_width = image.shape[:2]
        results = self.model(image, conf=0.5, verbose=False)
        annotated_frame = image.copy()

        result = results[0]
        objects = []
        send_data = []

        if result.masks and result.boxes:
            for i in range(len(result.boxes.data)):
                box = result.boxes[i]
                mask = result.masks.data[i]
                m = mask.cpu().numpy().astype(np.uint8)
                m = cv2.resize(m, (original_width, original_height), interpolation=cv2.INTER_LINEAR_EXACT)
                # m = mask_offset(m, (0, 0)) # offset if needed
                class_id = int(box.cls)
                object_data = {}
                object_data['name'] = self.model.names[class_id]
                object_data['mask'] = m
                object_data['color'] = self.colors.get(object_data['name'], (0, 255, 0))
                object_data['center'] = find_center(m)
                objects.append(object_data)
        draw_frame(annotated_frame, objects, show=False) # tính hệ số khoảng cách
        goc = get_object(objects, 'Goc')
        if goc is not None:
            for object in objects:
                name = object.get('name')
                if name is not None and name in ['Cam', 'Do', 'Vang']:
                    draw_contours(annotated_frame, object['mask'], object['color'], True)
                    display_object(annotated_frame, object)
                    x, y = get_coordinate(goc, object, annotated_frame)
                    x = np.round(x/100, 2) # mm sang m
                    y = np.round(y/100, 2)
                    send_data.append({'name': name, 'x': x, 'y': y})

        cv2.imshow('Camera Feed', annotated_frame)
        cv2.waitKey(1)

        if len(send_data):
            msg = String()
            msg.data = json.dumps(send_data)
            self.publisher_.publish(msg)
            self.get_logger().info(f'Sending: {msg.data}')

    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')
        image = self.cv_bridge.imgmsg_to_cv2(data, 'bgr8')
        self.yolo_detect(image)


def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber("cam_detect")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()