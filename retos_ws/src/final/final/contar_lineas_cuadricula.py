import cv2
import numpy as np
from time import sleep
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from semifinal.misfunciones import *
from final.Movements import Movements

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
        self.vid = cv2.VideoCapture(0)
        #self.vid = cv2.VideoCapture('/home/alemany/asti2024/retos_ws/src/semifinal/semifinal/video.mp4')

        self.estacionado = True
        self.giro = "der"
        self.counter = 50
        self.estado = 'Estacionado'
        self.veces_izquierda = 0
        self.veces_derecha = 0
        self.puntos_h = 4
        self.puntos_v = 4
        self.h = True
        self.v = True
        self.count = 0
        self.mov = Movements()

    def timer_callback(self):
        self.run()

    def check_for_black(self, region):
        return np.any(region < self.black_threshold)

    # MENÚ            
    def run(self):
        print("Menu:")
        print("1. Activar cámara sin procesamiento de imagen")
        print("2. Mover el robot")
        print("3. Ejecutar código existente")

        choice = input("Selecciona una opción (presiona 'q' para detener): ")

        if choice == 'q':
            self.stop_video_capture()
        elif choice == '1':
            self.activate_camera()
        elif choice == '2':
            self.move_robot()
        elif choice == '3':
            self.line_follower()
        else:
            print("Opción no válida. Inténtalo de nuevo.")

    # OPCIÓN 1: MOSTRAR CÁMARA
    def activate_camera(self):
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
            if cv2.waitKey(10) & 0xFF == ord('q'):
                break

    # OPCIÓN 2: MOVER ROBOT
    def move_robot(self):
        moves = [(0.0, 1.0), (0.0, -1.0), (0.1, 0.0), (-0.1, 0.0)]

        for move in moves:
            self.tupla = move
            sleep(1)
            msg = Twist()
            msg.linear.x = self.tupla[0]
            msg.angular.z = self.tupla[1]
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing: velocity="({msg.linear.x}, {msg.angular.z})"')
            self.tupla = (0.0, 0.0)
            sleep(1)
            msg = Twist()
            msg.linear.x = self.tupla[0]
            msg.angular.z = self.tupla[1]
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing: velocity="({msg.linear.x}, {msg.angular.z})"')
    
    # OPCIÓN 3: SIGUELINEAS (cambio nombre función para mayor aclaración)
    def line_follower(self):
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

            #cv2.imshow('frame', frame)
            
            # Dibujar líneas horizontales
            for i in range(1, 7):
                cv2.line(frame, (0, i * cell_size_y), (cols, i * cell_size_y), (0, 255, 0), 1)

	    # Dibujar líneas verticales
            for j in range(1, 7):
                cv2.line(frame, (j * cell_size_x, 0), (j * cell_size_x, rows), (0, 255, 0), 1)

            cv2.imshow('frame', frame)
            
            
            #Bobo
            for i in range(1, 7):
                cv2.line(frame, (0, i * cell_size_y), (cols, i * cell_size_y), (0, 255, 0), 1)

	    # Dibujar líneas verticales
            for j in range(1, 7):
                cv2.line(frame, (j * cell_size_x, 0), (j * cell_size_x, rows), (0, 255, 0), 1)

            cv2.imshow('frame', frame)
            
            
# Intersección a izquierda
            if (self.matrix[:, 0] == 1).any() and (self.matrix[:,3].all() == 1 or self.matrix[:,4].all() == 1) and self.h:  # TODO: Ajustable
                self.estacionado = False
                self.giro = 'interizq'
                if self.estado != 'interizq' and self.count >= 15:
                    self.puntos_h -= 1
                    self.count = 0      # No contar 2 veces una intersección
                self.estado = 'interizq'
                if self.puntos_h > 0:  # Modificar velocidad y reducirla antes del último punto
                    self.tupla = (0.5, 0.0) 
                elif self.puntos_h == 0:
                    self.estado = None      # No envia datos a los motores
                    if self.puntos_v > 0:
                        self.mov.girar_grados_der(90)
                    else:
                        self.mov.girar_grados_izq(90)
                else:
                    self.h = False  # Para no tener que entrar de nuevo al bucle
                    
                    
                            
            # Intersección a derecha
            elif (self.matrix[:, 6] == 1).any() and (self.matrix[:,3].all() == 1 or self.matrix[:,4].all() == 1) and self.v:
                self.estacionado = False
                self.giro = 'interder'
                if self.estado != 'interder' and self.count >= 15:
                    self.puntos_v -= 1
                    self.count = 0
                self.estado = 'interder'
                if self.puntos_v > 1:
                    self.tupla = (0.4, 0.0)
                elif self.puntos_v == 0:
                    self.tupla = (0.0, 0.0)
                else:
                    self.v = False
                    
                
            # Recto
            
            elif self.matrix[1:,3].all() == 1 or self.matrix[1:,4].all() == 1:
                self.tupla = (0.3, 0.0)
                self.estado = 'Recto'
            
            # Derecha
            elif self.matrix[0, 6] == 1 and self.matrix[0, 0] == 0:
                self.estacionado = False
                self.tupla = (0.3, -0.1)
                self.estado = 'Derecha'
                self.giro = 'der'
                self.veces_derecha += 1
                
            # Derecha 2
            elif self.matrix[1, 6] == 1 and self.matrix[1, 0] == 0:
                self.estacionado = False
                self.tupla = (0.2, -0.2)
                self.estado = 'Derecha'
                self.giro = 'der'
                self.veces_derecha += 1
            
            # Derecha 3
            elif self.matrix[2, 6] == 1 and self.matrix[2, 0] == 0:
                self.estacionado = False
                self.tupla = (0.1, -0.3)
                self.estado = 'Derecha'
                self.giro = 'der'
                self.veces_derecha += 1
                
            # Izquierda
            elif self.matrix[0, 0] == 1 and self.matrix[0, 6] == 0:
                self.estacionado = False
                self.tupla = (0.3, 0.1)
                self.estado = 'Izquierda1'
                self.giro = 'izq'
                self.veces_izquierda += 1
                
            # Izquierda 2
            elif self.matrix[1, 0] == 1 and self.matrix[1, 6] == 0:
                self.estacionado = False
                self.tupla = (0.3, 0.2)
                self.estado = 'Izquierda2'
                self.giro = 'izq'
                self.veces_izquierda += 1
            
            # Izquierda 3
            elif self.matrix[2, 0] == 1 and self.matrix[2, 6] == 0:
                self.estacionado = False
                self.tupla = (0.3, 0.3)
                self.estado = 'Izquierda3'
                self.giro = 'izq'
                self.veces_izquierda += 1            
           
            # Recto
            #elif self.matrix[0, 3] == 1 and self.matrix[0, 0] == 0 and self.matrix[0, 6] == 0:
                #self.estacionado = False
                #self.tupla = (0.3, 0.0)
                #self.estado = 'Recto'
            
            ## Recto
            #elif self.matrix[0, 0] == 1 and self.matrix[0, 6] == 1:
                #self.estacionado = False
                #self.tupla = (0.3, 0.0)
                #self.estado = 'Recto'
                
                
            # No detecta negro
            else:
                todos_cero = np.all(self.matrix == 0)
                if todos_cero:
                    self.estacionado = False
                    if self.giro == 'izq':
                        self.tupla = (0.0, 1.0)
                        self.estado = '+180º'
                    elif self.giro == 'der':
                        self.tupla = (0.0, -1.0)
                        self.estado = '-180º'
                    else:
                        self.tupla = (0.0, -1.0)
            


            # Publish tuple data
 
            msg = Twist()
            self.count += 1
            print(self.puntos_h) 
            print(self.matrix)
            if self.estado != None:
                msg.linear.x = self.tupla[0]
                msg.angular.z = self.tupla[1]
            print(self.estado)
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing: velocity="({msg.linear.x}, {msg.angular.z})"')
            
            # PULSAMOS C PARA QUE SIGA HACIA DELANTE FORZOSAMENTE CUANDO NOS CONVENGA (LA CÁMARA SE PARA PERO EN TEORIA DEBERÍA DAR IGUAL)
            if cv2.waitKey(10) & 0xFF == ord('w'):
                msg = Twist()
                msg.linear.x = 0.5
                msg.angular.z = 0.0
                print('Recto')
                self.publisher_.publish(msg)
                self.get_logger().info(f'Publishing: velocity="({msg.linear.x}, {msg.angular.z})"')
                sleep(0.5)
            elif cv2.waitKey(10) & 0xFF == ord('a'):
                msg = Twist()
                msg.linear.x = 0.0
                msg.angular.z = 0.5
                print('Izquierda')
                self.publisher_.publish(msg)
                self.get_logger().info(f'Publishing: velocity="({msg.linear.x}, {msg.angular.z})"')
                sleep(0.5)
            elif cv2.waitKey(10) & 0xFF == ord('d'):
                msg = Twist()
                msg.linear.x = 0.0
                msg.angular.z = -0.5
                print('Derecha')
                self.publisher_.publish(msg)
                self.get_logger().info(f'Publishing: velocity="({msg.linear.x}, {msg.angular.z})"')
                sleep(0.5)
            elif cv2.waitKey(10) & 0xFF == ord('x'):
                msg = Twist()
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                print('Derecha')
                self.publisher_.publish(msg)
                self.get_logger().info(f'Publishing: velocity="({msg.linear.x}, {msg.angular.z})"')
                self.vid.release()
                cv2.destroyAllWindows()
                rclpy.shutdown()
            sleep(0.1)
                

    # DETENER CAPTURA IMÁGEN
    def stop_video_capture(self):
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
