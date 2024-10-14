import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
#from semifinal.misfunciones import *

from sensor_msgs.msg import Image  # Image is the message type
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images

import keyboard


class DetectLinea(Node):
    def __init__(self, camara_sub=False):
        super().__init__('detectar_linea')

        # ================== Publisher ==================
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel_vel', 10)
        self.publisher_

        if camara_sub:
            self.publiscam_ = self.create_publisher(Image, 'camera/image_raw', 10)
            self.br = CvBridge()


        # ================== Subscriber ==================
        self.subscriber_ = self.create_subscription(Int32, 'rectificar', self.listener_callback, 10)

        self.subscriber_cam = self.create_subscription(Image, 'camera/image_raw', self.camara_callback, 10)

        timer_period = 0.001  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # ================== Variables ==================

        if camara_sub:
            self.cap = 0
        else:
            self.cap = cv2.VideoCapture(0)  # /home/jcrex/Vídeos/siguelineas_largo.mp4
            self.cap.set(3, 640)
            self.cap.set(4, 480)
        self.divisiones = 7

        self.contador = 0.15
        self.memoria = 0

        self.lower_blue = np.array([0, 0, 0])
        self.upper_blue = np.array([35, 35, 35])

        self.camara_sub = camara_sub

    def timer_callback(self):
        self.detectar()

    def listener_callback(self, msg: Int32):
        self.get_logger().info(f'Recibido: {msg.data}')
        self.memoria = msg.data

    def publish_velocity(self, velocity):            # velocity -->  tuple = (1, 0) (linear.x, angular.z)
        msg = Twist()
        msg.linear.x = velocity[0]
        msg.angular.z = velocity[1]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: velocity="({msg.linear.x}, {msg.angular.z})"')

    def camara_callback(self, msg: Image):
        self.camara_sub = True
        self.get_logger().info('Recibiendo video frame')
        self.cap = self.br.imgmsg_to_cv2(msg)

    def camara(self):
        # Uso para el testeo de camara
        while True:
            success, img = self.cap.read()
            if self.camara_sub:
                self.publishcamara(img)
            else:
                cv2.imshow("video", img)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break


    def publishcamara(self, img):
        self.publiscam_.publish(self.br.cv2_to_imgmsg(img))
        self.get_logger().info('Publishing video frame')

    def detectar_puntos_activos(self, cuadricula, hImg, wImg, img, puntos):
        """
        Detecta los puntos activos en la cuadrícula y los marca en la imagen.

        Args:
        - cuadricula: La cuadrícula de puntos activos.
        - hImg: La altura de la imagen.
        - wImg: El ancho de la imagen.
        - img: La imagen en la que se marcarán los puntos activos.
        - puntos: La matriz que almacena los puntos activos.
        """
        for i in range(self.divisiones):
            for j in range(self.divisiones):
                # Verificar si hay algún punto activo en la submatriz
                if np.any(cuadricula[i][j] > 0):
                    # Marcar el punto activo en la imagen con un círculo
                    cv2.circle(img, (
                    (j * 2 + 1) * wImg // (2 * self.divisiones), (i * 2 + 1) * hImg // (2 * self.divisiones)),
                               5, (50, 50, 255), 2)
                    # Asignar el valor correspondiente al punto activo en la matriz de puntos
                    if (i > (self.divisiones - 3) and j < 2) or (i < 2 and j < 2):
                        # Puntos en las esquinas tienen valor 2
                        puntos[i][j] = 2
                    elif (i > (self.divisiones - 3) and j > (self.divisiones - 3)) or (
                            i < 2 and j > (self.divisiones - 3)):
                        # Puntos en las esquinas tienen valor 2
                        puntos[i][j] = 2
                    else:
                        # Otros puntos tienen valor 1
                        puntos[i][j] = 1

    def calcular_diferencia_puntos(self, puntos):
        """
        Calcula la suma de puntos en la columna central y la diferencia de posiciones entre los lados derecho e izquierdo.

        Args:
        - puntos: la matriz que almacena los puntos activos.

        Returns:
        - suma_columna_central: la suma de puntos en la columna central.
        - diferencia_posiciones: la diferencia de posiciones entre los lados derecho e izquierdo.
        """
        suma_columna_central = sum(puntos[i][self.divisiones // 2] for i in range(self.divisiones))
        suma_derecha = sum(sum(puntos[i][self.divisiones // 2 + 1:]) for i in range(self.divisiones))
        suma_izquierda = sum(sum(puntos[i][:self.divisiones // 2 - 1]) for i in range(self.divisiones))
        diferencia_posiciones = suma_derecha - suma_izquierda
        return suma_columna_central, diferencia_posiciones

    def movimiento(self, suma_central, vel):
        """
        Controla el movimiento del robot en base a la suma de puntos en la columna central y la velocidad.

        Args:
        - suma_central: La suma de puntos en la columna central.
        - vel: La velocidad actual del robot.
        """
        # Definir la velocidad angular base
        velocidad_angular = 0.1

        # Caso 1: No hay puntos y la velocidad es cero
        if suma_central == vel == 0:
            if self.memoria == 0:
                # Mantener la trayectoria recta (si no hay memoria de giro previo)
                print("Mantener la trayectoria recta")
                self.publish_velocity((0.1, 0.0))
                #time.sleep(1)
            elif self.memoria == 1:
                # Girar hacia la izquierda (usando la memoria de giro previa)
                self.publish_velocity((0.0, -self.contador))
                self.contador -= 0.01
            elif self.memoria == -1:
                # Girar hacia la derecha (usando la memoria de giro previa)
                self.publish_velocity((0.0, self.contador))
                self.contador += 0.01

        # Caso 2: La velocidad es negativa (girar hacia la izquierda)
        elif vel < -13:
            if self.contador > 0:
                # Ajustar la velocidad angular para una rotación más suave
                print("Girar hacia la izquierda con mayor suavidad")
                self.publish_velocity((0.0, velocidad_angular - self.contador))
                self.contador -= 0.01
            else:
                # Giro estándar hacia la izquierda
                print("Girar hacia la izquierda")
                self.publish_velocity((0.0, velocidad_angular))
            self.memoria = -1

        # Caso 3: La velocidad es positiva (girar hacia la derecha)
        elif vel > 18:
            if self.contador > 0:
                # Ajustar la velocidad angular para una rotación más suave
                print("Girar hacia la derecha con mayor suavidad")
                self.publish_velocity((0.0, -velocidad_angular + self.contador))
                self.contador += 0.01
            else:
                # Giro estándar hacia la derecha
                print("Girar hacia la derecha")
                self.publish_velocity((0.0, -velocidad_angular))
            self.memoria = 1

        # Caso 4: Velocidad positiva o nula (ir recto)
        else:
            print("Ir recto")
            # Publicar velocidad para avanzar recto
            self.publish_velocity((0.18, 0.0))
            # Reiniciar el contador de ajuste de giro
            self.contador = 0.2

    def detectar(self):
        # Bucle principal para detectar puntos activos y realizar movimientos
        while True:
            # Leer un frame de la cámara
            success, img = self.cap.read()
            # Convertir el formato de color de BGR a RGB
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            # Convertir la imagen al espacio de color HSV
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            # Crear una máscara para identificar los píxeles dentro del rango de color azul
            mask = cv2.inRange(hsv, self.lower_blue, self.upper_blue)
            # Aplicar la máscara a la imagen original
            resultado = cv2.bitwise_and(img, img, mask=mask)

            # Obtener las dimensiones de la imagen resultante
            hImg, wImg, _ = resultado.shape

            # Dividir la imagen en una cuadrícula de divisiones x divisiones
            cuadricula = [
                [resultado[i * hImg // self.divisiones:(i + 1) * hImg // self.divisiones,
                 j * wImg // self.divisiones:(j + 1) * wImg // self.divisiones] for j in range(self.divisiones)]
                for i in range(self.divisiones)
            ]

            # Crear una matriz para almacenar la posición de los puntos activos
            puntos = [[0] * self.divisiones for _ in range(self.divisiones)]
            # Detectar los puntos activos en la cuadrícula
            self.detectar_puntos_activos(cuadricula, hImg, wImg, img, puntos)

            # Calcular la suma de puntos en la columna central y la diferencia de posiciones
            suma_central, diferencia = self.calcular_diferencia_puntos(puntos)
            # Imprimir información sobre la suma de puntos y la diferencia de posiciones
            #print("Suma en columna central:", suma_central)
            #print("Diferencia de posiciones:", diferencia)

            # Realizar el movimiento del robot en base a la suma de puntos y la diferencia de posiciones
            self.movimiento(suma_central, diferencia)

            # Mostrar las imágenes resultantes

            if self.camara_sub:
                self.publishcamara(img)
            else:
                #cv2.imshow("Result", resultado)
                #print("Mostrando imagen")
                cv2.imshow("Video", img)

            # Esperar a que se presione la tecla 'q' para salir del bucle


            if cv2.waitKey(1) & 0xFF == ord('s'):
                print("\n\n\n\n")
                self.publish_velocity((0.0, 0.0))
    
            if cv2.waitKey(1) & 0xFF == ord('w'):
                print("\n\n\n\n")
                self.memoria = 0
                self.publish_velocity((0.1, 0.0))
    
            if cv2.waitKey(1) & 0xFF == ord('a'):
                self.memoria = -1
                print("\n\n\n\n")
                self.publish_velocity((0.0, -0.5))
    
            if cv2.waitKey(1) & 0xFF == ord('d'):
                self.memoria = 1
                print("\n\n\n\n")
                self.publish_velocity((0.0, 0.5))

            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("Saliendo del bucle principal")
                break
    
            """

            if keyboard.is_pressed('s'):
                self.memoria = 0
            if keyboard.is_pressed('a'):
                self.memoria = -1
            if keyboard.is_pressed('d'):
                self.memoria = 1
            if keyboard.is_pressed('w'):
                self.memoria = 0
            if keyboard.is_pressed('q'):
                print("Saliendo del bucle principal")
                break
            """

        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()
        rclpy.shutdown()


def menu():

    opcion = int(input(
        "Menu de control del siguelineas elegir: \n"
        "1 para empezar a detectar linea y moverse \n"
        "2 para controlar los movimiento de forma manual \n"
        "3 Mostrar la camara y luego ejecutar codigo \n"
        "4 para modificar el color de la linea \n"
        "5 para salir \n"
    ))
    print(f"Opcion seleccionada {opcion}")
    return opcion


def main(args=None):

    rclpy.init(args=args)
    if input("esta en raspberry y/n ") == "y":
        robot = DetectLinea(camara_sub=True)
    else:
        robot = DetectLinea()

    opcion = menu()

    while opcion != 5:
        if opcion == 1:
            rclpy.spin(robot)
        elif opcion == 2:
            vel = int(input(
                "Elija la direccion que tomara: \n"
                "Valor < -13 girar a la izquierda \n"
                "Valor > 18 girar a la derecha \n"
                "Valor intermedio ir recto \n"
                "No poner nada acaba el programa \n"
            ))
            while vel != "":
                robot.movimiento(0, vel)
                vel = int(input(
                    "Elija la direccion que tomara: \n"
                    "Valor < -13 girar a la izquierda \n"
                    "Valor > 18 girar a la derecha \n"
                    "Valor intermedio ir recto \n"
                    "No poner nada acaba el programa \n"
                ))
            robot.publish_velocity((0.0, 0.0))

        elif opcion == 3:
            print("activada")
            robot.camara()
            rclpy.spin(robot)

        elif opcion == 4:
            robot.lower_blue = np.array(
                [int(input("Introduce el valor de azul minimo: ")), int(input("Introduce el valor de verde minimo: ")),
                 int(input("Introduce el valor de rojo minimo: "))])
            robot.upper_blue = np.array(
                [int(input("Introduce el valor de azul maximo: ")), int(input("Introduce el valor de verde maximo: ")),
                 int(input("Introduce el valor de rojo maximo: "))])
            rclpy.spin(robot)

    robot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
