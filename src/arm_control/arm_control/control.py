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


def vang_offset(x, y):
    x_offset = 0.5
    y_offset = -0.5
    if (y >= 3):
        x_offset -= 0.5
        if (x >= 15):
            x_offset -= 0.5
    if (y <= -1.9):
        x_offset += 1
        if (x <= 16):
            x_offset += 0.5
        if (x <= 11):
            x_offset += 0.5
    return x_offset, y_offset

def do_offset(x, y):
    x_offset = 0.8
    y_offset = 0
    if (y >= 3):
        if (x <= 13):
            x_offset += 0.5
    if (y <= -1.9):
        x_offset += 1
        if (x <= 16):
            x_offset += 0.5
        if (x <= 13):
            x_offset += 0.5
    return x_offset, y_offset

def cam_offset(x, y):
    x_offset = 0
    y_offset = 0.5
    if (y <= -1.9):
        x_offset += 1
    return x_offset, y_offset

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
        self.arm = RARM(channel=['9', '16', '19', '22', '24'], length=[14, 12, 9, 14])
        self.arm.set_limit(self.limit)
        self.dang_gap = False
        self.last_position = None

        # ARM config
        self.ban_dau = (10, 4, 30)
        self.khe_1 = (33, -4, 16)
        self.khe_2 = (33, 3, 16)
        self.khe_3 = (32, 12, 16)
        self.offset = {
            'Vang': {
                'xy': vang_offset,
                'g': 800,
            },
            'Do': {
                'xy': do_offset,
                'g': 700,
            },
            'Cam': {
                'xy': cam_offset,
                'g': 900,
            }
        }

        self.home()

        self.subscription = self.create_subscription(String, 'control', self.listener_callback, 1)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
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
                    self.get_logger().info(f'last_position: {self.last_position}, current: {name}, {x}, {y}')
                    if self.last_position is None or (self.last_position['name'] != name or self.last_position['x'] != x or self.last_position['y'] != y):
                        self.last_position = obj
                        self.dang_gap = True
                        self.get_logger().info(f'{name} at ({x}, {y})')
                        if name == 'Vang':
                            self.gap(x, y, name, self.khe_3)
                        elif name == 'Do':
                            self.gap(x, y, name, self.khe_2)
                        elif name == 'Cam':
                            self.gap(x, y, name, self.khe_1)

    def home(self):
        print("Going Home!")
        x, y, z = self.ban_dau
        self.arm.gotoXYZ(x, y, z, 0, gripper=450)
        sleep(0.5)

    def gap(self, x, y, name, slot):
        print(f'Đang gắp tại {x}, {y}')

        x_offset, y_offset = self.offset[name]['xy'](x, y)
        g = self.offset[name].get('g', 800)
        theta = np.radians(-60)
        # theta = np.radians(0)
        x_raw = x + 5
        x = x-2
        y -= 4
        y *= -1
        print(f"X: {x_raw}, Y: {y}")
        self.arm.gotoXYZ(x, y + y_offset, 6, theta, gripper=450, time=1000)
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
