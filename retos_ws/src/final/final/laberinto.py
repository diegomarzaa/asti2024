#!/usr/bin/env python3

# Import the necessary ROS2 libraries
import rclpy
import time
from math import pi, atan
from final.Movements import Movements

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

def pruebas(mov):
  print("Pruebas")
  mov.detectar_pared()

def ejecutar_laberinto(mov, opcion_menu):
  if opcion_menu == '1':
    pruebas(mov)
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
  mov.actualizar_vel_lineal(0.4)
  mov.actualizar_vel_angular(0.2)
  
  while True:
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
      ejecutar_laberinto(mov, opcion_menu)
    else:
      print("Opcion no valida, 'q' para salir")

  mov.detener()
  rclpy.shutdown()

