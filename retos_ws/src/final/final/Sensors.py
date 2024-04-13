import time
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from final.Movements import Movements

class Sensors(Node):
    def __init__(self):
        super().__init__('distance_sensor_subscriber')
        
        self.array_subscriber_ = self.create_subscription(Float32MultiArray, '/distancias_sensores', self.array_callback, 10)

    def array_callback(self, msg):

        def detectar_pared(sensor_del_izq, sensor_del_der, dist_pared = 20):
            print(f"Distancia delante izq: {sensor_del_izq}, Distancia delante der: {sensor_del_der}")
            if (sensor_del_izq < dist_pared) or (sensor_del_der < dist_pared):
                return True
            else:
                return False
            
        def avanzar_hasta_pared(mov, sensor_del_izq, sensor_del_der):
            if detectar_pared(sensor_del_izq, sensor_del_der):
                mov.detener()
            else:
                mov.avanzar_distancia(0.05)
            
        self.get_logger().info('Received array: ' + str(msg.data))

        mov = Movements()
        mov.actualizar_vel_lineal(0.2)
        distancia_der = msg.data[0]
        distancia_delante_izq = msg.data[1]
        distancia_delante_der = msg.data[2]
        distancia_der = msg.data[3]
        
        avanzar_hasta_pared(mov, distancia_delante_izq, distancia_delante_der)
        
    def get_distancia_derecha(self):
        return self.distancia_der
    
    def get_distancia_izquierda(self):
        return self.distancia_izq

    def get_distancia_delante_der(self):
        return self.distancia_delante_der
    
    def get_distancia_delante_izq(self):
        return self.distancia_delante_izq
    
    def get_distancias(self):
        """
        Return: tupla izquierda, delante_izq, delante_der, derecha
        """
        izquierda = self.get_distancia_izquierda()
        delante_izq = self.get_distancia_delante_izq()
        delante_der = self.get_distancia_delante_der()
        derecha = self.get_distancia_derecha()
        return (izquierda, delante_izq, delante_der, derecha)
    
    def principal(self):
        mov = Movements()
        while True:
            op = input("Introduce una opción (1-pruebas/2-salir): ")
            if op == '1':
                print("Pruebas")
                mov.detectar_pared(self)
            elif op == '2':
                break

    def detectar_pared(self, dist_pared = 20):
        dist_delante_izq = self.distancia_delante_izq()
        dist_delante_der = self.distancia_delante_der()
        print(f"Distancia delante izq: {dist_delante_izq}, Distancia delante der: {dist_delante_der}")
        if (dist_delante_izq < dist_pared) or (dist_delante_der < dist_pared):
            return True
        else:
            return False

def main(args=None):
    rclpy.init(args=args)
    sensors = Sensors()
    rclpy.spin(sensors)
    sensors.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

