import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

import os
os.environ['ROS_DOMAIN_ID'] = '173'
os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
os.environ['ROS_LOCALHOST_ONLY'] = '0'

class CameraPublisher(Node):

    def __init__(self, name):
        super().__init__(name)
        self.publisher_ = self.create_publisher(Image, 'image', 10)
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cv_bridge = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret == True:
            self.publisher_.publish(self.cv_bridge.cv2_to_imgmsg(frame, 'bgr8'))
            self.get_logger().info('Publishing video frame')

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher('cam_pub')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()