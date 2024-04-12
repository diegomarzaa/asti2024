#!/usr/bin/env python3

# Import the necessary ROS2 libraries
import rclpy
import time
from math import pi, atan
from final.Movements import Movements
from final.Sensors import Sensors


FACTOR = 2
DISTANCIA_ENTRE_SENSORES = 10.7

opciones = {
    '1': 'Pruebas',
    '2': 'Prueba mano derecha (opción 1)',
    '3': 'Prueba paralelo a paredes (opción 2)',
    '4': 'Prueba de laberinto básico (opción 3)',
    '5': 'Programa principal (sin acabar)',
    'q': 'Salir'
}

def girar_pared_diagonal(mov, sensors):
    distancia_delante_izq = sensors.get_distancia_delante_izq()
    distancia_delante_der = sensors.get_distancia_delante_der()
    y = abs(distancia_delante_izq - distancia_delante_der)
    x = DISTANCIA_ENTRE_SENSORES
    angulo_giro = atan(y/x)
    angulo_giro = angulo_giro * 180 / pi  
    if distancia_delante_der < distancia_delante_izq:
        mov.girar_grados_izq(angulo_giro)
    else:
        mov.girar_grados_der(angulo_giro)

def pedir_opcion_menu():
    print("\nOpciones laberinto:")
    for key, value in opciones.items():
        print(f" {key}. {value}")

    return input("Seleccione una opcion: ")

def pruebas(mov, sensors):
    print("Pruebas")
    mov.detectar_pared(sensors)

def basico(mov, sensors):
    # sensores.distancias() -> [izq, izq_f, der_f, der]
    distancias = sensors.distancias()
    if distancias[1] > 5 and distancias[2] > 5:
        if round(distancias[3],1) < 10:    # Demasiado cerca de la paredes
            mov.girar_der(10)
        elif round(distancias[3], 1) > 10 and distancias[3] < 30:
            mov.girar_izq(8)
        else:
            mov.avanzar()
        if distancias[3] > 30:
            mov.girar_grados_der(90)
    else:
        while round(distancias[2], 1) == round(distancias[1], 1) and distancias[1] < 25:
            mov.actualizar_vel_lineal(0.0)
            mov.actualizar_vel_angular(-0.1)
            distancias = sensors.distancias()
        mov.actualizar_vel_lineal(0.3)
        mov.actualizar_vel_angular(0.0)
            
    
def paralelo_paredes(mov, sensors): # Cambia el valor de las ruedas según la distancia a las paredes 
    distancias = sensors.distancias()
    diferencia = distancias[3] - distancias[0] # distancia entre derecha e izquierda
    if distancias[2] < 5 and distancias[1] < 5:
        while round(distancias[2], 1) == round(distancias[1], 1) and distancias[1] < 5:
            mov.actualizar_vel_lineal(0.0)
            mov.actualizar_vel_angular(-0.1)
            distancias = sensors.distancias()
        mov.actualizar_vel_lineal(0.3)
        mov.actualizar_vel_angular(0.0)
    if abs(diferencia) < 3:
        mov.actualizar_vel_angular(diferencia/FACTOR)    # Revisar signo y factor
    elif diferencia > 0:                            # Camino derecha
        mov.girar_grados_der(90, 0.2)
    elif distancias[1] < 25 and diferencia < 0:     # Única posibilida izquierda
        mov.girar_grados_izq(90, 0.2)
    else:
        mov.girar_grados_der(180)

def mano_derecha(mov, sensors):
    mov.avanzar_derecha(mov, sensors)


def ejecutar_laberinto(mov, sensors, opcion_menu):
    if opcion_menu == '1':
        pruebas(mov, sensors)
    elif opcion_menu == '2':
        mano_derecha(mov, sensors)
    elif opcion_menu == '3':
        paralelo_paredes(mov, sensors)
    elif opcion_menu == '4':
        basico(mov, sensors)
    elif opcion_menu == '5':
        main()
    elif opcion_menu == '6':
        pass
    else:
        print("Opcion no valida, 'q' para salir")

def main():
    rclpy.init(args=None)
    node = rclpy.create_node('laberinto')
    mov = Movements()
    sensors = Sensors()
    mov.actualizar_vel_lineal(0.4)
    mov.actualizar_vel_angular(0.2)
    
    
    while rclpy.ok():
        opcion_menu = pedir_opcion_menu()
        
        # Testeos
        if opcion_menu == 'a':
        mov.prueba_movimientos()
        elif opcion_menu == 'q':
        break

        # Cuadrícula
        elif opcion_menu in opciones:
        """
        1 -> Pruebas
        """
        rclpy.spin_once(sensors)    # ESTO MAMAHUEVO, ACTUALIZAR LOS SUSCRIBERS ANTES DE LLAMAR A NADA (Si está al final, no se actualizarán hasta que se llegue al final, en este caso aún no se ha llegado, por lo que sensors.distancia_der = None)
        ejecutar_laberinto(mov, sensors, opcion_menu)
        else:
        print("Opcion no valida, 'q' para salir")
        rclpy.spin_once(sensors)

    mov.detener()
    rclpy.shutdown()
