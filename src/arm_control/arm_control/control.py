import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import numpy as np
from .RARM_5DOF import RARM
from time import sleep

os.environ['ROS_DOMAIN_ID'] = '173'
os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
os.environ['ROS_LOCALHOST_ONLY'] = '0'

class ArmControl(Node):

    def __init__(self):
        super().__init__('ARM_control')

        # ARM setup
        self.limit = [
            {
                'min_pwm': 1500,
                'min_angle': np.radians(-25),
                'max_pwm': 2200,
                'max_angle': np.radians(35)
            },
            {
                'min_pwm': 950,
                'min_angle': 0,
                'max_pwm': 2400,
                'max_angle': np.radians(142),
            },
            {
                'min_pwm': 2500,
                'min_angle': np.radians(-146),
                'max_pwm': 820,
                'max_angle': 0,
            },
            {
                'min_pwm': 500,
                'min_angle': np.radians(-90),
                'max_pwm': 2500,
                'max_angle': np.radians(90)
            },
            {
                'min_pwm': 500,
                'min_angle': np.radians(-90),
                'max_pwm': 2500,
                'max_angle': np.radians(90)
            },
        ]

        self.arm = RARM(port='/dev/ttyACM0',
                        channel=['9', '16', '19', '22', '24'],
                        length=[14, 12, 9, 14])
        self.arm.set_limit(self.limit)
        self.dang_gap = False

        # ARM config
        self.ban_dau = (10, 0, 30)
        self.khe_1 = (33, -4, 16)
        self.khe_2 = (33, 3, 16)
        self.khe_3 = (32, 12, 16)
        self.offset = {
            'Vang': {
                'x': 5,
                'y': -0.5,
                'g': 800
            },
            'Do': {
                'x': 5.6,
                'y': 0,
                'g': 750
            },
            'Cam': {
                'x': 5,
                'y': 0.5,
                'g': 900
            }
        }
        self.home()

        self.subscription = self.create_subscription(
            String, 'control', self.listener_callback, 1)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        data = json.loads(msg.data)
        if data and len(data) > 0:
            if self.dang_gap:
                self.get_logger().info('Đang gắp, không thể gắp thêm!')
            else:
                obj = data[0]
                name = obj.get('name', None)
                x = obj.get('x', None)
                y = obj.get('y', None)
                if name and x and y:
                    self.get_logger().info(f'{name} at ({x}, {y})')
                    if name == 'Vang':
                        self.gap(x, y, name, self.khe_3)
                    elif name == 'Do':
                        self.gap(x, y, name, self.khe_2)
                    elif name == 'Cam':
                        self.gap(x, y, name, self.khe_1)
        self.get_logger().info('I heard: "%s"' % msg.data)

    def home(self):
        print("Going Home!")
        x, y, z = self.ban_dau
        self.arm.gotoXYZ(x, y, z, 0, gripper=450)
        sleep(0.5)

    def gap(self, x, y, name, slot):
        self.dang_gap = True
        print(f'Đang gắp tại {x}, {y}')

        x_offset = self.offset[name].get('x', 0)
        y_offset = self.offset[name].get('y', 0)
        g = self.offset[name].get('g', 800)
        theta = np.radians(-60)
        x_raw = x
        x = x-2
        x += 1
        y -= 3
        y *= -1
        self.arm.gotoXYZ(x, y + y_offset, 6, theta, gripper=450)
        sleep(0.5)
        self.arm.gotoXYZ(x_raw + x_offset, y + y_offset, 5, theta, gripper=450)
        sleep(0.5)
        self.arm.gotoXYZ(x_raw + x_offset, y + y_offset, 5, theta, gripper=g)
        sleep(0.5)
        self.arm.gotoXYZ(x_raw + x_offset, y + y_offset, 30, 0, gripper=g)
        sleep(0.5)

        x, y, z = slot
        self.arm.gotoXYZ(x, y, z, 0, gripper=g)
        sleep(0.5)
        self.arm.gotoXYZ(x, y, z, 0, gripper=450)
        sleep(0.5)

        self.home()
        self.dang_gap = False


def main(args=None):
    rclpy.init(args=args)

    node = ArmControl()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
