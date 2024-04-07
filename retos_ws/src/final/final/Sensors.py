import time
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
#from final.Movements import Movements

class Sensors(Node):
    def __init__(self):
        super().__init__('distance_sensor_subscriber')
        
        self.get_sensor_delante_izq_ = self.create_subscription(Float32, '/distance_delante_der', self.callback_delante_der, 10)
        self.get_sensor_delante_izq_
        self.get_sensor_delante_der_ = self.create_subscription(Float32, '/distance_delante_izq', self.callback_delante_izq, 10)
        self.get_sensor_delante_der_

        self.get_sensor_derecha_ = self.create_subscription(Float32, '/distance_der', self.callback_derecha, 10)
        self.get_sensor_derecha_
        self.get_sensor_izquierda_ = self.create_subscription(Float32, '/distance_izq', self.callback_izquierda, 10)
        self.get_sensor_izquierda_
        
        self.distancia_der = 0.0
        self.distancia_izq = 0.0
        self.distancia_delante_der = 0.0
        self.distancia_delante_izq = 0.0
        
    def callback_derecha(self, msg):
        self.distancia_der = msg.data
        
    def callback_izquierda(self, msg):
        self.distancia_izq = msg.data

    def callback_delante_der(self, msg):
        self.distancia_delante_der = msg.data
        
    def callback_delante_izq(self, msg):
        self.distancia_delante_izq = msg.data
        
    def get_distancia_derecha(self):
        return self.distancia_der
    
    def get_distancia_izquierda(self):
        return self.distancia_izq

    def get_distancia_delante_der(self):
        return self.distancia_delante_der
    
    def get_distancia_delante_izq(self):
        return self.distancia_delante_izq

def main(args=None):
    rclpy.init(args=args)
    sensors = Sensors()
    rclpy.spin(sensors)
    sensors.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
