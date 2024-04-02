import time
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class Sensors(Node):
    def __init__(self):
        super().__init__('distance_sensor_subscriber')
        self.distancia_der = None
        self.get_sensor_derecha_ = self.create_subscription(Float32, 'distance_der', self.callback_derecha, 10)
        self.get_sensor_derecha_
        
    def callback_derecha(self, msg):
        self.distancia_der = msg.data

    def detectar_pared(self):
        print(f'Distancia derecha: {self.distancia_der} cm')
        if self.distancia_der < 20:
            print("Muy cerca de un obstáculo")
            return True
        else:
            print("No hay obstáculos")
            return False

def main(args=None):
    rclpy.init(args=args)
    sensors = Sensors()
    rclpy.spin(sensors)
    sensors.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()