#!/usr/bin/env python3

# Import the necessary ROS2 libraries
import rclpy
import time
from math import pi, atan
from final.Movements import Movements
from final.Sensors import Sensors

DISTANCIA_TABLERO_COL = 1.2   # Suma de las distancias de las columnas
DISTANCIA_TABLERO_FILA = 1.05 # Suma de las distancias de las filas

COL_NUM = [0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15]
COL_LETRAS = [0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15]

FILA_NUM = [0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15]
FILA_LETRAS = [0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15]

opciones = {
  '1': 'Pruebas',
  'm': 'Mostrar coordenadas',
  'a': 'Prueba de movimiento',
  'q': 'Salir'
}

def pedir_opcion_menu():
  print("\nOpciones laberinto:")
  for key, value in opciones.items():
    print(f" {key}. {value}")
    
  return input("Seleccione una opcion: ")

def pruebas(mov, sensors):
  print("Pruebas")
  mov.detectar_pared(sensors)

def ejecutar_laberinto(mov, sensors, opcion_menu):
  if opcion_menu == '1':
    pruebas(mov, sensors)
  elif opcion_menu == '2':
    pass
  elif opcion_menu == '3':
    pass
  elif opcion_menu == '4':
    pass
  elif opcion_menu == '5':
    pass
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
