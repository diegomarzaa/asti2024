import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
import re


class LectorCmdVel(Node):
    def __init__(self):
        super().__init__('LectorCmdVel')
        self.msg = Twist()
        self.wheel_publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

    def publish_wheel_velocity(self, vel_x, vel_y):
        self.msg.linear.x = vel_x
        self.msg.angular.z = vel_y
        self.wheel_publisher_.publish(self.msg)
        self.get_logger().info(f'Publishing wheel velocity: {vel_x}, {vel_y}')

    def lectura(self):
        with open("cmd_vel", 'r') as archivo:
            s = archivo.read()
        return self.str_to_twist(s)

    def str_to_twist(self, s):
        # Extraer los valores de los componentes lineales y angulares
        matches = re.findall(r"[-+]?\d*\.\d+|\d+", s)
        print(matches)

        if len(matches) < 8:
            return None

        # Crear un nuevo mensaje Twist con los valores extraídos
        twist = Twist()
        twist.linear.x = float(matches[1])
        twist.linear.y = float(matches[2])
        twist.linear.z = float(matches[3])
        twist.angular.x = float(matches[5])
        twist.angular.y = float(matches[6])
        twist.angular.z = float(matches[7])

        return twist


def main():
    rclpy.init()
    lector = LectorCmdVel()
    while True:
        twist = lector.lectura()
        if twist is None:
            continue
        lector.publish_wheel_velocity(twist.linear.x, twist.angular.z)
    rclpy.shutdown()