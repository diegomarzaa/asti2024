#!/usr/bin/env python3

# Import the necessary ROS2 libraries
import rclpy
import time
from math import pi
from final.Movements import Movements, Servo     # Contiene los angulos de la herramienta

SLEEP_TIME_BOLI = 1
SLEEP_MOV = 0.1

DIST_BOLI_CENTRO = 0.12

VELOCIDAD_LINEAL = 0.05
VELOCIDAD_ANGULAR = 0.5

def get_figure_params(opcion_menu):
  figura = int(opcion_menu)
  
  if figura == 1: # Triangulo
    angulo = 133           # 2pi/3 radianes
    largo = 0.25
    ancho = 0.25
    lados = 3
  elif figura == 2: # Cuadrado
    angulo = 99           # 2pi/4   # TODO: Se queda corto 9 grados :(
    largo = 0.2
    ancho = 0.2
    lados = 4
  elif figura == 3: # Rectangulo
    angulo = 99            # 2pi/4
    largo = 0.3
    ancho = 0.1
    lados = 4
  else:
    raise ValueError("Invalid figure")
  return angulo, largo, ancho, lados



def dibujar_figura(mov, servo, opcion_menu):
  angulo, largo, ancho, lados = get_figure_params(opcion_menu)
  
  servo.boli_bajar()
  
  for i in range(lados):
    if i % 2:
      distancia = largo
    else:
      distancia = ancho
    
    print(f"Moviendo hacia adelante (pintando), distancia: {distancia}")
    mov.avanzar_distancia(distancia)
    servo.boli_subir(prints=True)
    time.sleep(SLEEP_TIME_BOLI)

    print(f"Avanzando para corregir (sin pintar) {DIST_BOLI_CENTRO}")
    mov.avanzar_distancia(DIST_BOLI_CENTRO)

    print(f"Girando {angulo} grados")
    mov.girar_grados_der(angulo)

    print("Moviendo hacia atras (sin pintar), {DIST_BOLI_CENTRO}")
    mov.retroceder_distancia(DIST_BOLI_CENTRO)

    print("Bajando boli")
    servo.boli_bajar(prints=True)



def main():
  rclpy.init(args=None)
  mov = Movements()
  servo = Servo()
          
  mov.actualizar_vel_lineal(VELOCIDAD_LINEAL)
  mov.actualizar_vel_angular(VELOCIDAD_ANGULAR)
  
  while True:
    
    print("\nFiguras:")
    print(" 1. Triangulo")
    print(" 2. Cuadrado")
    print(" 3. Rectangulo")
    print(" c. Calibrar boli")
    print(" q. Salir")
    opcion_menu = input("Seleccione una opcion: ")
    
    # OPCIONES
    
    if opcion_menu in ['1', '2', '3']:  # Triangulo, cuadrado, rectangulo
      dibujar_figura(mov, servo, opcion_menu)
      
    elif opcion_menu == 'c':
      servo.ver_angulos_herramienta()
      
    elif opcion_menu == 'q':
      break
    
    else:
      print("Opcion no valida, 'q' para salir")

  mov.detener()
  rclpy.shutdown()

