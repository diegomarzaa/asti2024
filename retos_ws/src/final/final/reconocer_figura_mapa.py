#!/usr/bin/env python3

# Import the necessary ROS2 libraries
import rclpy
import time
from math import pi, atan
from final.Movements import Movements
#from pruebas.pruebas_de_figuras import Figura

# Estaban dadas en mm pero aquí están pasadas a metros
COORDENADAS_CATEDRAL = (1.200, -0.248)   # TRIÁNGULO
COORDENADAS_MUSEO = (1.733, -0.036)       # RECTÁNGULO
COORDENADAS_UNIVERSIDAD = (0.035, -0.568)  # ARCO
COORDENADAS_CUBOS = (0.649, -0.213)       # CILINDRO
COORDENADAS_PLAZA_ESPAÑA = (1.687, 0.017) # ESTRELLA


def reconocer_figura(opcion_menu):
    """
    Se tendrá que importar la función correspondiente del programa de 
    reconocimiento de figuras implementado con visión artificial, no es
    mi oficio desarrollar esto ahora mismo
    """

    #figura = Figura()

    figura = dar_opcion(opcion_menu)
    return figura


def conseguir_objetivo(opcion_menu):

    figura = reconocer_figura(opcion_menu)

    if figura == 'triangulo':
        return COORDENADAS_CATEDRAL
    elif figura == 'rectangulo':
        return COORDENADAS_MUSEO
    elif figura == 'arco':
        return COORDENADAS_UNIVERSIDAD
    elif figura == 'cilindro':
        return COORDENADAS_CUBOS
    elif figura == 'estrella':
        return COORDENADAS_PLAZA_ESPAÑA
    else:
        print("Figura no reconocida")
        return None


def moverse_objetivo(mov, opcion_menu):

    objetivo = conseguir_objetivo(opcion_menu)

    if objetivo is not None:
        y, x = objetivo

        # CALCULAMOS EL ÁNGULO
        angulo = atan(y/abs(x))
        angulo = angulo * 180 / pi
        angulo_giro = 90 - angulo

        if x > 0:
            angulo_giro = -angulo_giro

        # CALCULAMOS LA DISTANCIA QUE TIENE QUE AVANZAR
        distancia = (x**2 + y**2)**0.5

        mov.girar_avanzar(angulo_giro, distancia)

    else:
        print("No se ha podido conseguir el objetivo")


def pedir_opcion_menu():
    print("\nFiguras:")
    print(" 1. Triangulo")
    print(" 2. Rectangulo")
    print(" 3. Arco")
    print(" 4. Cilindro")
    print(" 5. Estrella")
    print(" ----------------- ")
    print(" a. Testeo")
    print(" q. Salir")
    return input("Seleccione una opcion: ")

def dar_opcion(opcion_menu):
    if opcion_menu == '1':
        return "triangulo"
    elif opcion_menu == '2':
        return "rectangulo"
    elif opcion_menu == '3':
        return "arco"
    elif opcion_menu == '4':
        return "cilindro"
    elif opcion_menu == '5':
        return "estrella"
    else:
        return None


def main():
    rclpy.init(args=None)
    node = rclpy.create_node('reconocimiento_figuras_mapa')
    mov = Movements()

    while True:
        opcion_menu = pedir_opcion_menu()

        # Testeos
        if opcion_menu == 'a':
            mov.prueba_movimientos()
        elif opcion_menu == 'q':
            break

        # Cuadrícula
        elif opcion_menu in ['1', '2', '3', '4', '5']:
            """
            1 -> triangulo - CATEDRAL
            2 -> rectangulo - MUSEO
            3 -> arco - UNIVERSIDAD
            4 -> cilindro - CUBOS
            5 -> estrella - PLAZA ESPAÑA
            """
            moverse_objetivo(mov, opcion_menu)
        else:
            print("Opcion no valida, 'q' para salir")

    mov.detener()
    rclpy.shutdown()

