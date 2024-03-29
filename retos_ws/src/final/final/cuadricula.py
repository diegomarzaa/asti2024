#!/usr/bin/env python3

# Import the necessary ROS2 libraries
import rclpy
import time
from math import pi, atan
from final.Movements import Movements


COL_NUM = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
COL_LETRAS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

FILA_NUM = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
FILA_LETRAS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def pedir_opcion_menu():
  print("\nOpciones cuadrícula:")
  print(" 1. Ángulo a ojo")
  print(" 2. Cálculo con trigonometría")
  print(" 3. Posición final")
  print(" 4. Trampa zapato (falta por implementar)")
  print(" 5. Contar líneas (falta por implementar)")
  print(" 6. Ángulo a ojo + reconocimiento de letras (falta por implementar)")
  print(" u. Actualizar coordenadas")
  print(" m. Mostrar coordenadas")
  print("\nPruebas:")
  print(" a. Prueba de movimiento")
  print(" q. Salir")
  
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
  print("Columnas:")
  for i in range(8):
    print(f"{i}-{i+1}: {COL_LETRAS[i]}")
  print("Filas:")
  for i in range(7):
    print(f"{i}-{i+1}: {FILA_LETRAS[i]}")

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

def posicion_final(mov):
  
    # INTRODUCIMOS LA POSICIÓN INICIAL DEL NÚMERO Y LA FINAL DE LA LETRA
    inicio = int(input("Introduce la posición inicial (-: columna(9 posibles) / +: fila(8 posibles)): "))
    fin = int(input("Introduce la posición final (-: columna(9 posibles) / +: fila(8 posibles)): "))

    # CALCULAMOS LAS DISTANCIAS DE LAS LETRAS Y NÚMEROS
    distancia_numero = 0
    if inicio < 0:
      for i in range(inicio-1):
          distancia_numero += COL_LETRAS[i]
    else:
      for i in range(inicio-1):
          distancia_numero += FILA_LETRAS[i]

    distancia_letra = 0
    if fin < 0:
      for i in range(fin-1):
          distancia_letra += COL_NUM[i]
    else:
      for i in range(fin-1):
          distancia_letra += FILA_NUM[i]

    # CALCULAMOS LA DISTANCIA X E Y
          
    # Inicio - columna números , Fin - columna letras
    if inicio < 0 and fin < 0:
      y = 2.5
      x = distancia_letra - distancia_numero

    # Inicio - fila números , Fin - fila letras
    elif inicio > 0 and fin > 0:
      x = 2.5
      y = distancia_letra - distancia_numero

    # Inicio - columna números , Fin - fila letras
    elif inicio < 0 and fin > 0:
      x = 2.5 - distancia_numero
      y = 2.5 - distancia_letra

    # Inicio - fila números , Fin - columna letras
    elif inicio > 0 and fin < 0:
      x = distancia_letra
      y = distancia_numero
    
    # CALCULAMOS EL ÁNGULO
    angulo = atan(abs(y)/abs(x))
    angulo = angulo * 180 / pi
    angulo_giro = 90 - angulo

    if x > 0 or y > 0:
      angulo_giro = -angulo_giro
    
    # CALCULAMOS LA DISTANCIA QUE TIENE QUE AVANZAR
    distancia = (x**2 + y**2)**0.5
  
    # GIRAMOS Y AVANZAMOS
    mov.girar_avanzar(angulo_giro, distancia)

def ejecutar_cuadricula(mov, opcion_menu):
  if opcion_menu == '1':
    angulo_ojo(mov)
  elif opcion_menu == '2':
    calculo_trigonometria(mov)
  elif opcion_menu == '3':
    posicion_final(mov)
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
  node = rclpy.create_node('cuadricula')
  mov = Movements()
  
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
    elif opcion_menu in ['1', '2', '3', '4', '5', '6']:
      """
      1 -> Ángulo a ojo
      2 -> Cálculo con trigonometría
      3 -> Posición final
      4 -> Trampa zapato
      5 -> Contar líneas
      6 -> Ángulo a ojo + reconocimiento de letras
      """
      ejecutar_cuadricula(mov, opcion_menu)
    else:
      print("Opcion no valida, 'q' para salir")

  mov.detener()
  rclpy.shutdown()

