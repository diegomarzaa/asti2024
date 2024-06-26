import time
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from final.Movements import Movements

DISTANCIA_ENTRE_SENSORES = 10.7

class Sensors(Node):
    def __init__(self):
        super().__init__('distance_sensor_subscriber')
        
        self.array_subscriber_ = self.create_subscription(Float32MultiArray, '/distancias_sensores', self.array_callback, 10)

    def array_callback(self, msg):

        def girar_pared_diagonal(mov, sensor_del_izq, sensor_del_der):
            y = abs(sensor_del_izq - sensor_del_der)
            x = DISTANCIA_ENTRE_SENSORES
            angulo_giro = math.atan(y/x)
            angulo_giro = angulo_giro * 180 / math.pi  
            if distancia_delante_der < distancia_delante_izq:
                mov.girar_grados_izq(angulo_giro)
            else:
                mov.girar_grados_der(angulo_giro)

        def detectar_pared(sensor_del_izq, sensor_del_der, dist_pared = 40):
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

        def detectar_izquierda_libre(sensor_izq, dist_izq_libre = 35):
            if sensor_izq > dist_izq_libre:
                return True
            else:
                return False

        def detectar_derecha_libre(sensor_der, dist_der_libre = 35):
            if sensor_der > dist_der_libre:
                return True
            else:
                return False

        def avanzar_hasta_pared_izquierda(mov, sensor_izq):
            while True:
                if detectar_izquierda_libre(sensor_izq):
                    break
                mov.avanzar_distancia(0.05)
            mov.detener()
            
        self.get_logger().info('Received array: ' + str(msg.data))

        mov = Movements()
        mov.actualizar_vel_lineal(0.1)
        distancia_izq = msg.data[0]
        distancia_delante_izq = msg.data[1]
        distancia_delante_der = msg.data[2]
        distancia_der = msg.data[3]
        
        mov.avanzar_distancia(0.04)
        print("--RECTO--")
        if detectar_izquierda_libre(distancia_izq):
            print("*izquierda libre*")
            mov.detener()
            time.sleep(1)
            mov.girar_grados_izq(90)
            mov.avanzar_distancia(0.4)
            time.sleep(1)
        elif detectar_pared(distancia_delante_izq, distancia_delante_der):
            print("*pared*")
            mov.detener()
            time.sleep(1)
            if detectar_izquierda_libre(distancia_izq):
                print("*correccion: izquierda libre*")
                mov.girar_grados_izq(90)
                mov.avanzar_distancia(0.4)
            elif abs(distancia_delante_izq - distancia_delante_der) < 10:
                if distancia_izq > distancia_der:
                    mov.girar_grados_izq(45)
                else:
                    mov.girar_grados_der(45)
                mov.avanzar_distancia(0.1)
            else:
                girar_pared_diagonal(mov, distancia_delante_izq, distancia_delante_der)
            time.sleep(1)
        elif detectar_derecha_libre(distancia_der):
            print("*derecha libre*")
            mov.detener()
            time.sleep(1)
            mov.girar_grados_der(90)
            mov.avanzar_distancia(0.4)
            time.sleep(1)
        """
        elif distancia_izq < 10:
            print("reajuste, izquierda muy cerca")
            mov.detener()
            mov.girar_grados_der(10)
            time.sleep(1)
        elif distancia_der < 15:
            print("reajuste, derecha muy cerca")
            mov.detener()
            mov.girar_grados_izq(10)
            time.sleep(1)
        """
        


def main(args=None):
    rclpy.init(args=args)
    sensors = Sensors()
    rclpy.spin(sensors)
    sensors.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

