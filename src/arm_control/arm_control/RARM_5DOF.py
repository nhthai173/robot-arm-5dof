from serial import Serial, PARITY_NONE, STOPBITS_ONE, EIGHTBITS
from time import sleep
import numpy as np

class RARM:
    lastPos = [0, 0, 0, 0, 0]
    def __init__(self, channel = ['9', '16', '19', '22', '24'], port = '/dev/ttyACM0', length = [14, 12, 9, 14]):
        """Initialize 5 DoF Robot arm
        Args:
            port (str, optional): Defaults to '/dev/ttyACM0' for jetson(ubuntu) or 'COMx' for windows.
            channel (list, optional): Defaults to ['9', '16', '19', '22', '24'].
            length (list, optional): Defaults to [14, 12, 9, 14].
        """
        self.limit = [{}, {}, {}, {}, {}]
        self.port = port
        self.length = length
        self.l1 = length[0]
        self.l2 = length[1]
        self.l3 = length[2]
        self.l4 = length[3]
        self.set_channel(channel)
        self.serial = Serial(port = self.port, baudrate = 115200, parity = PARITY_NONE, stopbits = STOPBITS_ONE, bytesize = EIGHTBITS, timeout = 1)
        if not self.serial.is_open:
            self.serial.open()

    def set_channel(self, channel):
        """
        Set the channel for the servos.
        Args:
            channel (list): A list of 5 elements representing the channels for the servos.
        Returns:
            None
        """
        if channel is None:
            return
        if len(channel) != 5:
            raise ValueError("Channel must be a list of 5 elements.")
        self.channel = channel

    def set_port(self, port):
        """
        Set the port for the serial connection.
        Args:
            port (str): The port to set.
        Returns:
            None
        """
        if port is None:
            return
        self.port = port
        if self.serial is not None and self.serial.is_open:
            self.serial.close()
        self.serial = Serial(port = self.port, baudrate = 115200, parity = PARITY_NONE, stopbits = STOPBITS_ONE, bytesize = EIGHTBITS, timeout = 1)

    def set_length(self, length):
        """
        Set the lengths of the robot arm segments.
        Args:
            length (list): A list of 4 elements representing the lengths of the arm segments.
        Returns:
            None
        """
        if length is None:
            return
        if len(length) != 4:
            raise ValueError("Length must be a list of 4 elements.")
        self.length = length
        self.l1 = length[0]
        self.l2 = length[1]
        self.l3 = length[2]
        self.l4 = length[3]

    def set_limit(self, limit):
        """
        Set the limits for each servo.
        Args:
            limit (list): A list of 5 dictionaries, each containing 'min_pwm', 'min_angle', 'max_pwm', and 'max_angle'.
        Returns:
            None
        """
        if limit is None:
            return
        if len(limit) != 5:
            raise ValueError("Limit must be a list of 5 elements.")
        for i, lm in enumerate(limit):
            if 'min_pwm' not in lm or 'min_angle' not in lm:
                raise ValueError("Minimum limit must have 'min_pwm' and 'min_angle' keys.")
            if 'max_pwm' not in lm or 'max_angle' not in lm:
                raise ValueError("Maximum limit must have 'max_pwm' and 'max_angle' keys.")
        self.limit = limit

    def _mapVal(self, inVal, inMin, inMax, outMin, outMax):
        return int((inVal - inMin) * (outMax - outMin) / (inMax - inMin) + outMin)
    
    def _pwmCalc(self, angle_index, angle_value):
        if angle_index >= len(self.limit):
            return 0
        min_pwm = self.limit[angle_index]['min_pwm']
        min_angle = self.limit[angle_index]['min_angle']
        max_pwm = self.limit[angle_index]['max_pwm']
        max_angle = self.limit[angle_index]['max_angle']
        if angle_value < min_angle:
            angle_value = min_angle
        if angle_value > max_angle:
            angle_value = max_angle
        pwm = self._mapVal(angle_value, min_angle, max_angle, min_pwm, max_pwm)
        return pwm
    
    def _angle_to_pwm(self, angle_index, angle_value):
        if angle_index == 4:
            return 500
        return self._pwmCalc(angle_index, angle_value)
    
    def _inverse_kinematics(self, xe, ye, ze, theta, elbow_config = "down"):
        theta1 = np.arctan2(ye, xe)
        
        # the end point of joint 3
        x_eff = xe - self.l4 * np.cos(theta) * np.cos(theta1)
        y_eff = ye - self.l4 * np.cos(theta) * np.sin(theta1)
        z_eff = ze - self.l4 * np.sin(theta)
        
        r_eff = np.sqrt(x_eff**2 + y_eff**2)
        d = np.sqrt(r_eff**2 + (z_eff - self.l1)**2)
        
        if d > (self.l2 + self.l3) or d < abs(self.l2 - self.l3):
            raise ValueError("Position not reachable")

        cos_theta3 = (d**2 - self.l2**2 - self.l3**2) / (2 * self.l2 * self.l3)
        cos_theta3 = np.clip(cos_theta3, -1.0, 1.0)
        theta3_positive = np.arccos(cos_theta3)  # [0, pi]
        theta3_negative = -np.arccos(cos_theta3)  # [-pi, 0]
        
        if np.abs(cos_theta3) > 0.99:  # Gần thẳng hàng
            raise ValueError("Configuration is close to straight line")
        
        theta3 = theta3_positive if elbow_config == "up" else theta3_negative
        
        beta = np.arctan2(self.l3 * np.sin(theta3), self.l2 + self.l3 * np.cos(theta3))
        phi = np.arctan2(z_eff - self.l1, r_eff)
        theta2 = phi - beta

        theta4 = theta - (theta2 + theta3)
                
        print(f"IK: theta1={np.degrees(theta1):.2f}, theta2={np.degrees(theta2):.2f}, "
            f"theta3={np.degrees(theta3):.2f}, theta4={np.degrees(theta4):.2f}")
        return theta1, theta2, theta3, theta4
    
    def setPos(self, pos, time = 500, delay = 500):
        """Set position of the robot arm"
        Args:
            pos (list[int]): list of 5 elements, pwm values for each servo.
            time (int, optional): Defaults to 500.
            delay (int, optional): Defaults to 500.
        """
        buf = ''
        for i in range(len(pos)):
            self.lastPos[i] = pos[i]
            buf += f'#{self.channel[i]}P{pos[i]}'
        buf+= f'T{time}D{delay}\r\n'
        self.serial.write(buf.encode())
        sleep(time/1000)
        sleep(1e-3)

    def gotoXYZ(self, x, y, z, theta = -np.pi/4, gripper=500, elbow_config = 'down'):
        """
        Move the robot arm to the specified position.
        Args:
            x (float): X coordinate.
            y (float): Y coordinate.
            z (float): Z coordinate.
            theta (float, optional): Orientation angle in radians.
            gripper (int, optional): Gripper Servo PWM. Defaults to 500.
            elbow_config (str, optional): Elbow configuration ('up' or 'down'). Defaults to 'down'.
        """
        if theta < -np.pi or theta > np.pi:
            theta = np.radians(theta)
        t1, t2, t3, t4 = self._inverse_kinematics(x, y, z, theta, elbow_config)
        p1, p2, p3, p4 = self._angle_to_pwm(0, t1), self._angle_to_pwm(1, t2), self._angle_to_pwm(2, t3), self._angle_to_pwm(3, t4)
        print(f"PWM: p1={p1}, p2={p2}, p3={p3}, p4={p4}")
        self.setPos([p1, p2, p3, p4, gripper])

    