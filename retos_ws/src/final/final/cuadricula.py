#!/usr/bin/env python3

# Import the necessary ROS2 libraries
import rclpy
import time
from math import pi, atan
from final.Movements import Movements


def pedir_opcion_menu():
  print("\nOpciones cuadrícula:")
  print(" 1. Ángulo a ojo")
  print(" 2. Cálculo con trigonometría")
  print(" 3. Trampa zapato (falta por implementar)")
  print(" 4. Contar líneas (falta por implementar)")
  print(" 5. Ángulo a ojo + reconocimiento de letras (falta por implementar)")
  print("\nPruebas:")
  print(" a. Prueba de movimiento")
  print(" q. Salir")
  
  return input("Seleccione una opcion: ")

def angulo_ojo(mov):

  # INTRODUCIMOS EL ÁNGULO QUE QUEREMOS GIRAR
  angulo = float(input("Introduce el ángulo (- izq / + der): "))

  # INTRODUCIMOS LA DISTANCIA QUE QUEREMOS AVANZAR
  distancia = float(input("Introduce la distancia: "))

  # GIRAMOS Y AVANZAMOS
  mov.girar_avanzar(angulo, distancia)

def calculo_trigonometria(mov):

  # INTRODUCIMOS LA DISTANCIA X E Y, ADEMÁS DE LA DIRECCIÓN DE GIRO
  x = float(input("Introduce la distancia x (cm, la distancia hacia su lado): ")) / 100
  y = float(input("Introduce la distancia y (cm, lo que le falta avanzar) ")) / 100
  giro = input("Dirección de giro (izq/der) (grados): ")

  # CALCULAMOS EL ÁNGULO
  angulo = atan(y/x)
  angulo = angulo * 180 / pi
  angulo_giro = 90 - angulo

  # CALCULAMOS LA DISTANCIA QUE TIENE QUE AVANZAR
  distancia = (x**2 + y**2)**0.5

  # GIRAMOS Y AVANZAMOS
  if giro == 'izq':
    mov.girar_avanzar(angulo_giro, distancia)
  elif giro == 'der':
    mov.girar_avanzar(-angulo_giro, distancia)
  else:
    print("Dirección de giro no válida")

def ejecutar_cuadricula(mov, opcion_menu):
  if opcion_menu == '1':
    angulo_ojo(mov)
  elif opcion_menu == '2':
    calculo_trigonometria(mov)
  elif opcion_menu == '3':
    pass
  elif opcion_menu == '4':
    pass
  elif opcion_menu == '5':
    pass
  else:
    print("Opcion no valida, 'q' para salir")

def main():
  rclpy.init(args=None)
  node = rclpy.create_node('cuadricula')
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
      1 -> Ángulo a ojo
      2 -> Cálculo con trigonometría
      3 -> Trampa zapato
      4 -> Contar líneas
      5 -> Ángulo a ojo + reconocimiento de letras
      """
      ejecutar_cuadricula(mov, opcion_menu)
    else:
      print("Opcion no valida, 'q' para salir")

  mov.detener()
  rclpy.shutdown()

