import cv2
import imutils
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image  # Image is the message type
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images

from math import pi, atan

from final.Movements import Movements


class Figura(Node):
    """
    Clase que se encarga de detectar figuras geométricas en una imagen y de mover el robot hacia el objetivo
    correspondiente a la figura detectada.

    Attributes
    ----------
    br : CvBridge
        Objeto que permite convertir entre mensajes de ROS y objetos de OpenCV.
    mov : Movements
        Objeto que permite controlar los movimientos del robot.
    tonos : dict
        Diccionario que contiene los valores de tonos mínimos para cada gama de colores.
    formas : dict
        Diccionario que almacena el número de veces que se ha detectado cada figura geométrica.
    tiempo : float
        Tiempo transcurrido desde el inicio de la detección de figuras.
    seleccion : bool
        Indica si se ha seleccionado una opción del menú.

    seleccion : str
        Indica la opción seleccionada del menú.

    autonomia : bool
        Indica si el robot se moverá automáticamente o no.

    Methods
    -------
    listener_callback(msg)
        Callback que se ejecuta cada vez que se recibe un mensaje de tipo Image.
    detect(contour)
        Dada una figura geométrica, determina cuál es la forma más cercana a la misma.
    analizar(image)
        Dada una imagen, detecta las figuras geométricas presentes en la misma.
    conseguir_objetivo(figura)
        Dada una figura geométrica, retorna las coordenadas correspondientes al objetivo de dicha figura.
    moverse_objetivo(mov, figura)
        Dada una figura geométrica, mueve el robot hacia el objetivo correspondiente a dicha figura.
    pedir_opcion_menu()
        Muestra un menú con las figuras geométricas disponibles y solicita al usuario que seleccione una.
    dar_opcion(opcion_menu)
        Dada una opción del menú, retorna la figura geométrica correspondiente.

    Notes
    -----
    La funcion pedir_opcion_menu() no tiene uso en el código actual, pero se deja por si se quiere hacer cambios.

    La implementacion de trampas en el codigo, no creo que sea necesario porque el programa en ultima instancia
    no discrimina en formas, sino por el numero de veces que se detecta una forma, ademas, el programa detecta
    todas las formas que se le presentan, por lo que no se necesita hacer trampas.

    El programa depende de la iluminacion de la imagen, por lo que se necesita una buena iluminacion para que
    el programa funcione correctamente.

    El codigo entero puede ser completamente automatizado, pero se ha dejado la opcion de seleccionar una figura
    para que el usuario pueda ver el funcionamiento del programa.

    """

    def __init__(self, autonomia=True):
        super().__init__('Figura')  # /video_frames  /camera/image_raw
        self.subscription = self.create_subscription(Image, '/video_frames', self.listener_callback, 10)
        self.subscription

        self.br = CvBridge()
        self.mov = Movements()

        self.tonos = {
            "Oscuros": [1, 40],
            "Gama de oscuros": [40, 80],
            # 0    1    2    3    4    5    6   7     8    9   10   11   12   13
            "Tibios": [110, 120, 130, 140, 150, 160, 170, 180, 190, 200, 210, 220, 230, 240,
                       # 14   15   16   17   18   19   20   21   22   23   24   25   26   27
                       115, 125, 135, 145, 155, 165, 175, 185, 195, 205, 215, 225, 235, 245],
            "Claros": [250, 254],
            "Pruebas": [100.5]
        }

        self.formas = {
            "TRIANGULO": 0,
            "CUADRADO": 0,
            "ARCO": 0,
            "CILINDRO": 0,
            "CIRCULO": 0,
            "ESTRELLA": 0
        }

        self.tiempo = 0.0

        self.seleccion = True
        self.autonomia = autonomia

    def listener_callback(self, msg):
        # self.get_logger().info('Receiving video frame')

        # La opcion_menu de momento no tiene uso pero se deja por si se quiere hacer cambios

        if type(self.seleccion) is bool and self.seleccion:
            opcion_menu = self.pedir_opcion_menu()
            self.seleccion = opcion_menu

        # Testeos
        if self.seleccion == 'a':
            self.mov.prueba_movimientos()
        elif self.seleccion == 'q':
            print("cerrando")
            self.destroy_node()
            self.mov.destroy_node()
            rclpy.shutdown()
            cv2.destroyAllWindows()

        # Cuadrícula
        elif self.seleccion in ['0','1', '2', '3', '4', '5']:
            """
            1 -> triangulo - CATEDRAL
    rclpy.spin(linea_sub)         2 -> rectangulo - MUSEO
            3 -> arco - UNIVERSIDAD
            4 -> cilindro - CUBOS
            5 -> estrella - PLAZA ESPAÑA
            """
            # Convert ROS Image message to OpenCV image
            img = self.br.imgmsg_to_cv2(msg)

            # Analizamos la imagen
            self.analizar(img)

            # Aumentamos el tiempo
            self.tiempo += 0.1

            # Si no estamos en autonomia, pedimos al usuario que seleccione una figura
            if not self.autonomia:
                if self.seleccion == "1":
                    self.moverse_objetivo(self.mov, "TRIANGULO")
                    self.seleccion = True
                elif self.seleccion == "2":
                    self.moverse_objetivo(self.mov, "CUADRADO")
                    self.seleccion = True
                elif self.seleccion == "3":
                    self.moverse_objetivo(self.mov, "ARCO")
                    self.seleccion = True
                elif self.seleccion == "4":
                    self.moverse_objetivo(self.mov, "CILINDRO")
                    self.seleccion = True
                elif self.seleccion == "5":
                    self.moverse_objetivo(self.mov, "ESTRELLA")
                    self.seleccion = True

            elif self.tiempo >= 5.0:
                # Obtenemos la forma con mayor frecuencia
                maximo = max(self.formas, key=self.formas.get)
                print("\n\n\n\n\n\n\n\n\n", maximo, "\n\n\n\n\n\n\n\n\n")

                # Pedimos al usuario que presione cualquier boton para arrancar el robot
                _ = input("Presion cualquier boton para arrancar")

                # Movemos el robot hacia el objetivo
                self.moverse_objetivo(self.mov, maximo)

                # Reseteamos los valores
                self.tiempo = 0.0
                self.formas = {
                    "TRIANGULO": 0,
                    "CUADRADO": 0,
                    "ARCO": 0,
                    "CILINDRO": 0,
                    "CIRCULO": 0,
                    "ESTRELLA": 0
                }

                # Activamos que el usuario seleccione una figura
                self.seleccion = True

            # Display image
            cv2.imshow("camera", img)
            cv2.waitKey(1)

        else:
            print("Opcion no valida, 'q' para salir")

    def detect(self, contour):
        """
        Función que, dado un contorno, retorna la forma geométrica más cercana con base al número de lados del perímetro del
        mismo.
        :param contour: Contorno del que inferiremos una figura geométrica.
        :return: Texto correspondiente a la figura geométrica identificada (TRIANGULO, CUADRADO, RECTANGULO, PENTAGONO o CIRCULO)
        """
        # Hallamos el perímetro (cerrado) del contorno.
        perimeter = cv2.arcLength(contour, True)

        # Aproximamos un polígono al contorno, con base a su perímetro.
        approximate = cv2.approxPolyDP(contour, .03 * perimeter, True)

        print(len(approximate))

        # Si el polígono aproximado tiene 3 lados, entonces es un triángulo.
        if len(approximate) == 3:
            shape = 'TRIANGULO'
            self.formas["TRIANGULO"] += 1
        # Si el polígono aproximado tiene 4 lados, entonces puede ser o un cuadrado o un rectángulo.
        elif len(approximate) == 4:
            # Calculamos la relación de aspecto.
            x, y, w, h = cv2.boundingRect(approximate)
            aspect_ratio = w / float(h)

            # La figura será un cuadrado si la relación de aspecto está entre 95% y 105%, es decir, si todos los lados miden
            # más o menos lo mismo. En caso contrario, se trata de un rectángulo.
            if .95 <= aspect_ratio <= 1.05:
                shape = 'CUADRADO'
                self.formas["CUADRADO"] += 1
            else:
                shape = 'CUADRADO'
                self.formas["CUADRADO"] += 1
        # Si el polígono aproximado tiene 5 lados, es un pentágono.
        elif len(approximate) == 5:
            shape = 'ARCO'
            self.formas["ARCO"] += 1

        # Si el polígono aproximado tiene entre 9 y 10 lados, es una estrella.
        elif len(approximate) >= 9 and len(approximate) <= 10:
            shape = 'estrella'
            self.formas["ESTRELLA"] += 1
        # Si el polígono aproximado tiene entre 7 y 8 lados, es un círculo.
        elif 7 <= len(approximate) <= 8:
            shape = 'circulo'
            self.formas["CIRCULO"] += 1
        # En cualquier otro caso, se considera un cilindro.
        else:
            shape = 'CILINDRO'
            self.formas["CILINDRO"] += 1

        return shape

    def analizar(self, image):

        # Cargamos la imagen de entrada y la redimensionamos.
        resized = imutils.resize(image, width=380)

        # Calculamos la relación de proporción entre la imagen original y la redimensionada.
        ratio = image.shape[0] / float(resized.shape[0])

        # Convertimos la imagen a escala de grises, la difuminamos y la binarizamos.
        gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (3, 3), 0)  # Mejor resultado con (1, 1), 1

        cv2.imshow("blurred", blurred)
        cv2.waitKey(1)

        # =============================================================================
        # =============================================================================
        # =============================================================================

        # El valor tonos indica la minima media de colores que permite, si se indica 30 tomara todos los colores que
        # tenga una media de color RGB por encima de 30

        thresholded = cv2.threshold(blurred, self.tonos["Tibios"][0], 255, cv2.THRESH_BINARY)[1]
        cv2.imshow("thresholded", thresholded)
        cv2.waitKey(1)

        # Invierte los colores de la imagen
        inverted_image = cv2.bitwise_not(thresholded)
        cv2.imshow("Inverted Image", inverted_image)
        cv2.waitKey(1)

        thresholded = inverted_image

        # =============================================================================
        # =============================================================================
        # =============================================================================

        # Hallamos los contornos.
        contours = cv2.findContours(thresholded.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = imutils.grab_contours(contours)

        # Iteramos sobre cada contorno...
        for contour in contours:
            # Calculamos los momentos del contorno para encontrar su centro.
            M = cv2.moments(contour)
            # print(M, contour)
            try:
                center_x = int((M['m10'] / M['m00']) * ratio)
                center_y = int((M['m01'] / M['m00']) * ratio)
            except ZeroDivisionError:
                continue

            # Determinamos la forma geométrica del contorno usando la función que definimos previamente.
            shape = self.detect(contour)

            # Ajustamos el contorno a la imagen original.
            contour = contour.astype('float') * ratio
            contour = contour.astype('int')

            # Dibujamos el contorno y lo etiquetamos con el nombre de la figura geométrica identificada.
            cv2.drawContours(image, [contour], -1, (0, 255, 0), 2)
            cv2.putText(image, shape, (center_x, center_y), cv2.FONT_HERSHEY_SIMPLEX, .7, (255, 0, 0), 2)

            # Mostramos el contorno en la imagen original.
            # cv2.imshow('Imagen', image)
            # cv2.waitKey(0)

    def conseguir_objetivo(self, figura):

        # Estaban dadas en mm pero aquí están pasadas a metros
        COORDENADAS_CATEDRAL = (1.200, -0.248)  # TRIÁNGULO
        COORDENADAS_MUSEO = (1.733, -0.036)  # RECTÁNGULO
        COORDENADAS_UNIVERSIDAD = (0.035, -0.568)  # ARCO
        COORDENADAS_CUBOS = (0.649, -0.213)  # CILINDRO
        COORDENADAS_PLAZA_ESPAÑA = (1.687, 0.017)  # ESTRELLA

        # self.detectado = self.analizar(img)
        # figura = self.detectado

        if figura == 'TRIANGULO':
            return COORDENADAS_CATEDRAL
        elif figura == 'CUADRADO':
            return COORDENADAS_MUSEO
        elif figura == 'ARCO':
            return COORDENADAS_UNIVERSIDAD
        elif figura == 'CILINDRO':
            return COORDENADAS_CUBOS
        elif figura == 'ESTRELLA':
            return COORDENADAS_PLAZA_ESPAÑA
        else:
            print("Figura no reconocida")
            return None

    def moverse_objetivo(self, mov, figura):

        objetivo = self.conseguir_objetivo(figura)

        if objetivo is not None:
            y, x = objetivo

            # CALCULAMOS EL ÁNGULO
            angulo = atan(y / abs(x))
            angulo = angulo * 180 / pi
            angulo_giro = 90 - angulo

            if x > 0:
                angulo_giro = -angulo_giro

            # CALCULAMOS LA DISTANCIA QUE TIENE QUE AVANZAR
            distancia = (x ** 2 + y ** 2) ** 0.5

            mov.girar_avanzar(angulo_giro, distancia)

        else:
            print("No se ha podido conseguir el objetivo")

    def pedir_opcion_menu(self):

        print("\nFiguras:"
                "\n 0. Autonomia"
                "\n 1. Triangulo"
                "\n 2. Cuadrado"
                "\n 3. Arco"
                "\n 4. Cilindro"
                "\n 5. Estrella"
                "\n ----------------- "
                "\n a. Testeo"
                "\n q. Salir")

        return input("Seleccione una opcion: ")

    def dar_opcion(self, opcion_menu):
        if opcion_menu == '1':
            return "TRIANGULO"
        elif opcion_menu == '2':
            return "CUADRADO"
        elif opcion_menu == '3':
            return "ARCO"
        elif opcion_menu == '4':
            return "CILINDRO"
        elif opcion_menu == '5':
            return "ESTRELLA"
        else:
            return None


def main(args=None):
    rclpy.init(args=args)
    linea_sub = Figura(input("Autonomia: (y/n)") == "y")
    rclpy.spin(linea_sub)
    linea_sub.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
