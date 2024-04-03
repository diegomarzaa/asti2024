import time
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
#from final.Movements import Movements

class Sensors(Node):
    def __init__(self):
        super().__init__('distance_sensor_subscriber')
        self.distancia_der = None
        self.distancia_izq = None
        
        self.get_sensor_derecha_ = self.create_subscription(Float32, 'distance_der', self.callback_derecha, 10)
        self.get_sensor_derecha_
        self.get_sensor_izquierda_ = self.create_subscription(Float32, 'distance_izq', self.callback_izquierda, 10)
        self.get_sensor_izquierda_
        
    def callback_derecha(self, msg):
        self.distancia_der = msg.data
        
    def callback_izquierda(self, msg):
        self.distancia_izq = msg.data

    def get_distancia_derecha(self):
        return self.distancia_der
    
    def get_distancia_izquierda(self):
        return self.distancia_izq


def main(args=None):
    rclpy.init(args=args)
    sensors = Sensors()
    rclpy.spin(sensors)
    sensors.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()