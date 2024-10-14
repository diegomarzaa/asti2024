import cv2
import numpy as np
from time import sleep
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from semifinal.misfunciones import *

class LineaPublisher(Node):

    def __init__(self):
        super().__init__('linea_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.publisher_
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.tupla = (0.0, 0.0)
        self.matrix = np.zeros((7, 7), dtype=int)
        self.black_threshold = 40
        self.vid = cv2.VideoCapture(2)
        #self.vid = cv2.VideoCapture('/home/alemany/asti2024/asti2024_ws/src/semifinal/semifinal/video.mp4')

        self.estacionado = True
        self.giro = "der"
        self.counter = 50
        self.estado = 'Estacionado'
        self.veces_izquierda = 0
        self.veces_derecha = 0

    def timer_callback(self):

        self.run()

    def check_for_black(self, region):
        """
        frame = cv2.cvtColor(region, cv2.COLOR_BGR2RGB)
        # cv2.circle(frame, (320, 480), 5, (50, 50, 255), 2)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([0, 0, 0])
        upper_blue = np.array([100, 100, 100])

        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        frame = cv2.bitwise_and(frame, frame, mask=mask)
        """
        return np.any(region < self.black_threshold)

    def run(self):
        while True:
            try:
                ret, frame = self.vid.read()
                rows, cols, _ = frame.shape
            except AttributeError:
                break

            cell_size_x = cols // 7
            cell_size_y = rows // 7
            for i in range(7):
                for j in range(7):
                    roi = frame[i * cell_size_y:(i + 1) * cell_size_y, j * cell_size_x:(j + 1) * cell_size_x]
                    if self.check_for_black(roi):
                        self.matrix[i, j] = 1
                    else:
                        self.matrix[i, j] = 0

            cv2.imshow('frame', frame)
            
            # Dibujar líneas horizontales
            for i in range(1, 7):
                cv2.line(frame, (0, i * cell_size_y), (cols, i * cell_size_y), (0, 255, 0), 1)

	    # Dibujar líneas verticales
            for j in range(1, 7):
                cv2.line(frame, (j * cell_size_x, 0), (j * cell_size_x, rows), (0, 255, 0), 1)

            cv2.imshow('frame', frame)
            
            
            #Bobo
            if self.veces_derecha >= 3 and self.veces_izquierda >= 3: 
                self.estacionado = False
                self.tupla = (0.1, 0.0)
                self.estado = 'Recto'
                self.veces_derecha = 0
                self.veces_izquierda = 0
                
            # Derecha
            elif self.matrix[0, 6] == 1 and self.matrix[0, 0] == 0:
                self.estacionado = False
                self.tupla = (0.0, -0.2)
                self.estado = 'Derecha'
                self.giro = 'der'
                self.veces_derecha += 1
                
            # Derecha 2
            elif self.matrix[1, 6] == 1 and self.matrix[1, 0] == 0:
                self.estacionado = False
                self.tupla = (0.0, -0.3)
                self.estado = 'Derecha'
                self.giro = 'der'
                self.veces_derecha += 1
            
            # Derecha 3
            elif self.matrix[2, 6] == 1 and self.matrix[2, 0] == 0:
                self.estacionado = False
                self.tupla = (0.0, -0.4)
                self.estado = 'Derecha'
                self.giro = 'der'
                self.veces_derecha += 1
                
            # Izquierda
            elif self.matrix[0, 0] == 1 and self.matrix[0, 6] == 0:
                self.estacionado = False
                self.tupla = (0.0, 0.2)
                self.estado = 'Izquierda'
                self.giro = 'izq'
                self.veces_izquierda += 1
                
            # Izquierda 2
            elif self.matrix[1, 0] == 1 and self.matrix[1, 6] == 0:
                self.estacionado = False
                self.tupla = (0.0, 0.3)
                self.estado = 'Izquierda'
                self.giro = 'izq'
                self.veces_izquierda += 1
            
            # Izquierda 3
            elif self.matrix[2, 0] == 1 and self.matrix[2, 6] == 0:
                self.estacionado = False
                self.tupla = (0.0, 0.4)
                self.estado = 'Izquierda'
                self.giro = 'izq'
                self.veces_izquierda += 1            
           
            # Recto
            elif self.matrix[0, 3] == 1 and self.matrix[0, 0] == 0 and self.matrix[0, 6] == 0:
                self.estacionado = False
                self.tupla = (0.5, 0.0)
                self.estado = 'Recto'
            
            # Recto
            elif self.matrix[0, 0] == 1 and self.matrix[0, 6] == 1:
                self.estacionado = False
                self.tupla = (0.5, 0.0)
                self.estado = 'Recto'
                
            # 180º
            elif self.matrix[0, 3] == 0 and self.matrix[0, 0] == 0 and self.matrix[0, 6] == 0 and self.matrix[
                3, 3] == 1 and \
                    self.matrix[6, 3] == 1:
                self.estacionado = False
                if self.giro == 'izq':
                    self.tupla = (0.0, 1.0)
                    self.estado = '+180º'
                elif self.giro == 'der':
                    self.tupla = (0.0, -1.0)
                    self.estado = '-180º'
                    
            # Intersección a izquierda
            elif self.matrix[0, 0] == 1 and self.matrix[0, 6] == 1 and self.matrix[6, 0] == 1 and self.matrix[
                6, 6] == 0:
                self.estacionado = False
                self.giro = 'izq'
            
            # Intersección a derecha
            elif self.matrix[0, 0] == 1 and self.matrix[0, 6] == 1 and self.matrix[6, 0] == 0 and self.matrix[
                6, 6] == 1:
                self.estacionado = False
                self.giro = 'der'
                
            # No detecta negro
            else:
                todos_cero = np.all(self.matrix == 0)
                if todos_cero:
                    if self.giro == 'izq':
                        self.tupla = (0.0, 1.0)
                        self.estado = '+180º'
                    elif self.giro == 'der':
                        self.tupla = (0.0, -1.0)
                        self.estado = '-180º'

            # Publish tuple data
            msg = Twist()
            msg.linear.x = self.tupla[0]
            msg.angular.z = self.tupla[1]
            print(self.estado)
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing: velocity="({msg.linear.x}, {msg.angular.z})"')

            if cv2.waitKey(10) & 0xFF == ord('q'):
                self.vid.release()
                cv2.destroyAllWindows()
                rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    linea_pub = LineaPublisher()
    rclpy.spin(linea_pub)
    linea_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
