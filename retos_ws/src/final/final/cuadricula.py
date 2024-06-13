#!/usr/bin/env python3

# Import the necessary ROS2 libraries
import rclpy
import time
from math import pi, atan
from final.Movements import Movements, pedir_velocidades

ERROR_ANGULO = 0.0        # TODO: Comprobar cual es el error sistematico
DISTANCIA_EXTRA = 0.2

"""
DISTANCIA_TABLERO_COL = 2.87   # Suma de las distancias de las columnas
DISTANCIA_TABLERO_FILA = 2.83 # Suma de las distancias de las filas

COL_NUM = [0.34, 0.11, 0.63, 0.52, 0.46, 0.24, 0.23, 0.34]
COL_LETRAS = [0.34, 0.11, 0.63, 0.52, 0.46, 0.24, 0.23, 0.34]

FILA_NUM = [0.40, 0.33, 0.50, 0.14, 0.745, 0.215, 0.50]
FILA_LETRAS = [0.40, 0.33, 0.50, 0.14, 0.745, 0.215, 0.50]
"""

DISTANCIA_TABLERO_COL = 2.47   # Suma de las distancias de las columnas
DISTANCIA_TABLERO_FILA = 2.43 # Suma de las distancias de las filas

COL_NUM = [0.23, 0.46, 0.20, 0.26, 0.17, 0.53, 0.35, 0.24]
COL_LETRAS = [0.23, 0.46, 0.20, 0.26, 0.17, 0.53, 0.35, 0.24]

FILA_NUM = [0.37, 0.215, 0.58, 0.38, 0.515, 0.22, 0.15]
FILA_LETRAS = [0.37, 0.215, 0.58, 0.38, 0.515, 0.22, 0.15]

# INICIO - NÚMEROS
diccionario_numeros = {
                      '1': -2, '2': -3, '3': -4, '4': -5, '5': -6, '6': -7, '7': -8, '8': -9,   # COLUMNAS - (9 POSIBLES)
                      '9': +2, '10': +3, '11': +4, '12': +5, '13': +6, '14': +7, '15': +8             # FILAS +  (8 POSIBLES)       
                      }

# FIN - LETRAS
diccionario_letras = {
                      'A': -2, 'B': -3, 'C': -4, 'D': -5, 'E': -6, 'F': -7, 'G': -8, 'H': -9,   # COLUMNAS - (9 POSIBLES)
                      'I': +2, 'J': +3, 'K': +4, 'L': +5, 'M': +6, 'N': +7, 'O': +8             # FILAS +  (8 POSIBLES)       
                      }

opciones = {
  '1': 'Ángulo a ojo',
  '2': 'Cálculo con trigonometría',
  '3': 'Posición final',
  'u': 'Actualizar coordenadas',
  'm': 'Mostrar coordenadas',
  'a': 'Prueba de movimiento',
  'v': 'Actualizar velocidades',
  'w': 'Actualizar error ángulo',
  'q': 'Salir'
}

def pedir_opcion_menu():
  print("\nOpciones cuadrícula:")
  for key, value in opciones.items():
    print(f" {key}. {value}")
    
  return input("Seleccione una opcion: ")

def actualizar_coordenadas():
  global COL_NUM, COL_LETRAS, FILA_NUM, FILA_LETRAS

  # NÚMEROS
  for i in range(8):
    COL_NUM[i] = float(input(f"Introduce la columna {i}-{i+1}: "))

  for i in range(7):
    FILA_NUM[i] = float(input(f"Introduce la fila {i}-{i+1}: "))

  # LETRAS
  for i in range(8):
    COL_LETRAS[i] = float(input(f"Introduce la columna {i}-{i+1}: "))

  for i in range(7):
    FILA_LETRAS[i] = float(input(f"Introduce la fila {i}-{i+1}: "))

def mostrar_coordenadas():
  print("Números:")
  print("\n\tColumnas:")
  for i in range(8):
    print(f"\t\t{i}-{i+1}: {COL_NUM[i]}")
  print("\n\tFilas:")
  for i in range(7):
    print(f"\t\t{i}-{i+1}: {FILA_NUM[i]}")

  print("Letras:")
  print("\n\tColumnas:")
  for i in range(8):
    print(f"\t\t{i}-{i+1}: {COL_LETRAS[i]}")
  print("\n\tFilas:")
  for i in range(7):
    print(f"\t\t{i}-{i+1}: {FILA_LETRAS[i]}")
    
def actualizar_error_angulo():
  global ERROR_ANGULO
  print(f"Error actual: {ERROR_ANGULO}")
  error_a_poner = input("Introduce el error del ángulo (Enter para omitir): ")
  if error_a_poner:
    ERROR_ANGULO = float(error_a_poner)

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
  angulo = angulo * 180 / pi  + ERROR_ANGULO      #TODO: REVISAR
  angulo_giro = 90 - angulo

  # CALCULAMOS LA DISTANCIA QUE TIENE QUE AVANZAR
  distancia = (x**2 + y**2)**0.5    +  DISTANCIA_EXTRA 

  # GIRAMOS Y AVANZAMOS
  if giro == 'izq':
    mov.girar_avanzar(angulo_giro, distancia)
  elif giro == 'der':
    mov.girar_avanzar(-angulo_giro, distancia)
  else:
    print("Dirección de giro no válida")

def posicion_final(mov):
  
    # INTRODUCIMOS LA POSICIÓN INICIAL DEL NÚMERO Y LA FINAL DE LA LETRA
    inicio_num = input("Introduce la posición inicial (NÚMERO): ")
    fin_letra = input("Introduce la posición final (LETRA): ")

    inicio = diccionario_numeros[inicio_num]
    fin = diccionario_letras[fin_letra]

    # CALCULAMOS LAS DISTANCIAS DE LAS LETRAS Y NÚMEROS
    distancia_numero = 0
    if inicio < 0:
      for i in range(abs(inicio)-1):
          distancia_numero += COL_LETRAS[i]
    else:
      for i in range(inicio-1):
          distancia_numero += FILA_LETRAS[i]

    distancia_letra = 0
    if fin < 0:
      for i in range(abs(fin)-1):
          distancia_letra += COL_NUM[i]
    else:
      for i in range(fin-1):
          distancia_letra += FILA_NUM[i]

    # CALCULAMOS LA DISTANCIA X E Y
          
    # Inicio - columna números , Fin - columna letras
    # Inicio - fila números , Fin - fila letras
    if (inicio < 0 and fin < 0) or (inicio > 0 and fin > 0):
      if inicio < 0:
        y = DISTANCIA_TABLERO_FILA
      else:
        y = DISTANCIA_TABLERO_COL
      x = distancia_letra - distancia_numero

    # Inicio - columna números , Fin - fila letras
    elif inicio < 0 and fin > 0:
      x = DISTANCIA_TABLERO_COL - distancia_numero
      y = DISTANCIA_TABLERO_FILA - distancia_letra

    # Inicio - fila números , Fin - columna letras
    elif inicio > 0 and fin < 0:
      x = distancia_numero
      y = -distancia_letra
    
    # CALCULAMOS EL ÁNGULO
    if x == 0:
      angulo_giro = 0
    else:
      angulo = atan(abs(y)/abs(x))
      angulo = angulo * 180 / pi    + ERROR_ANGULO
      angulo_giro = 90 - angulo

    if x < 0 or y < 0:
      angulo_giro = -angulo_giro
    
    # CALCULAMOS LA DISTANCIA QUE TIENE QUE AVANZAR
    distancia = (x**2 + y**2)**0.5    + DISTANCIA_EXTRA
  
    # GIRAMOS Y AVANZAMOS
    print(f"X: {x}, Y: {y}")
    print(f"Ángulo: {angulo_giro}, Distancia: {distancia}")
    mov.girar_avanzar(angulo_giro, distancia)

def ejecutar_cuadricula(mov, opcion_menu):
  if opcion_menu == '1':
    angulo_ojo(mov)
  elif opcion_menu == '2':
    calculo_trigonometria(mov)
  elif opcion_menu == '3':
    posicion_final(mov)
  elif opcion_menu == 'v':
    pedir_velocidades(mov)
  elif opcion_menu == 'w':
    actualizar_error_angulo()
  else:
    print("Opcion no valida, 'q' para salir")

def main():
  rclpy.init(args=None)
  node = rclpy.create_node('cuadricula')
  mov = Movements()
  mov.actualizar_vel_lineal(0.4)
  mov.actualizar_vel_angular(0.2)
  
  while True:
    opcion_menu = pedir_opcion_menu()
    
    # Testeos
    if opcion_menu == 'a':
      mov.prueba_movimientos()
    if opcion_menu == 'u':
      actualizar_coordenadas()
    if opcion_menu == 'm':
      mostrar_coordenadas()
    elif opcion_menu == 'q':
      break

    # Cuadrícula
    elif opcion_menu in opciones:
      """
      1 -> Ángulo a ojo
      2 -> Cálculo con trigonometría
      3 -> Posición final
      """
      ejecutar_cuadricula(mov, opcion_menu)
    else:
      print("Opcion no valida, 'q' para salir")

  mov.detener()
  rclpy.shutdown()

