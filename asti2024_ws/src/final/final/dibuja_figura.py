#!/usr/bin/env python3

# Import the necessary ROS2 libraries
import rclpy
import time
from math import pi
from final.Movements import Movements, Servo     # Contiene los angulos de la herramienta

SLEEP_TIME_BOLI = 1
SLEEP_MOV = 0.1

DIST_BOLI_CENTRO = 0.18

VELOCIDAD_LINEAL = 0.1
VELOCIDAD_ANGULAR = 0.5

TIEMPO_TESTEOS = 1

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


class Dibujar:
  def __init__(self):
    self.grados_dibujar_arriba = 15.0
    self.grados_dibujar_abajo = 0.0
    self.tiempo_dibujar_arriba = 0.2
    self.tiempo_dibujar_abajo = 0.2
    self.servo = Servo()
    
  def boli_subir(self, prints=False):
    if prints:
      print(f"Boli subir: {self.grados_dibujar_arriba} grados")
    self.servo.herramienta_girar(self.grados_dibujar_arriba, self.tiempo_dibujar_arriba, final_tranquilo=True)
    
  def boli_bajar(self, prints=False):
    if prints:
      print(f"Boli bajar: {self.grados_dibujar_abajo} grados")
    self.servo.herramienta_girar(self.grados_dibujar_abajo, self.tiempo_dibujar_abajo, final_tranquilo=True)

    


def main():
  rclpy.init(args=None)
  mov = Movements()
  dibujar = Dibujar()
          
  mov.actualizar_vel_lineal(VELOCIDAD_LINEAL)
  mov.actualizar_vel_angular(VELOCIDAD_ANGULAR)
  
  while True:
    
    print("\nFiguras:")
    print(" 1. Triangulo")
    print(" 2. Cuadrado")
    print(" 3. Rectangulo")
    print("\nOpciones:")
    print(" 0. Posicion 0")
    print(" c. Calibrar boli")
    
    print(" q. Salir")
    opcion_menu = input("Seleccione una opcion: ")
    
    
    
    
    # OPCIONES
    
    if opcion_menu in ['1', '2', '3']:  # Triangulo, cuadrado, rectangulo
      
      if opcion_menu != "1":
        angulo, largo, ancho, lados = get_figure_params(opcion_menu)
        
        dibujar.boli_bajar()
        
        for i in range(lados):
          if i % 2:
            distancia = largo
          else:
            distancia = ancho
          
          print(f"Moviendo hacia adelante (pintando), distancia: {distancia}")
          mov.avanzar_distancia(distancia)
          dibujar.boli_subir(prints=True)
          time.sleep(SLEEP_TIME_BOLI)

          print(f"Avanzando para corregir (sin pintar) {DIST_BOLI_CENTRO}")
          mov.avanzar_distancia(DIST_BOLI_CENTRO)

          print(f"Girando {angulo} grados")
          mov.girar_grados_der(angulo)

          print(f"Moviendo hacia atras (sin pintar), {DIST_BOLI_CENTRO}")
          mov.retroceder_distancia(DIST_BOLI_CENTRO)

          print("Bajando boli")
          dibujar.boli_bajar(prints=True)
          
          
      # TRIANGULO BUG
      
      if opcion_menu == "1":
        angulo, largo, ancho, lados = get_figure_params(opcion_menu)
        
        dibujar.boli_bajar()
        distancia = largo
        
        # 1
        
        print(f"Moviendo hacia adelante (pintando), distancia: {distancia}")
        mov.avanzar_distancia(distancia)
        dibujar.boli_subir(prints=True)
        time.sleep(SLEEP_TIME_BOLI)

        print(f"Avanzando para corregir (sin pintar) {DIST_BOLI_CENTRO}")
        mov.avanzar_distancia(DIST_BOLI_CENTRO)

        print(f"Girando {angulo} grados")
        mov.girar_grados_der(angulo)

        print(f"Moviendo hacia atras (sin pintar), {DIST_BOLI_CENTRO}")
        mov.retroceder_distancia(DIST_BOLI_CENTRO-0.1)

        print("Bajando boli")
        dibujar.boli_bajar(prints=True)
        
        # 2
        
        print(f"Moviendo hacia adelante (pintando), distancia: {distancia}")
        mov.avanzar_distancia(distancia)
        dibujar.boli_subir(prints=True)
        time.sleep(SLEEP_TIME_BOLI)

        print(f"Avanzando para corregir (sin pintar) {DIST_BOLI_CENTRO}")
        mov.avanzar_distancia(DIST_BOLI_CENTRO)

        print(f"Girando {angulo} - 27 grados")
        mov.girar_grados_der(angulo - 27)

        print(f"Moviendo hacia atras (sin pintar), {DIST_BOLI_CENTRO}")
        mov.retroceder_distancia(DIST_BOLI_CENTRO)

        print("Bajando boli")
        dibujar.boli_bajar(prints=True)

	      # 3
       
        print(f"Moviendo hacia adelante (pintando), distancia: {distancia}")
        mov.avanzar_distancia(distancia)
        dibujar.boli_subir(prints=True)
        time.sleep(SLEEP_TIME_BOLI)

        print(f"Avanzando para corregir (sin pintar) {DIST_BOLI_CENTRO}")
        mov.avanzar_distancia(DIST_BOLI_CENTRO)

        print(f"Girando {angulo} - 10 grados")
        mov.girar_grados_der(angulo - 10)

        print(f"Moviendo hacia atras (sin pintar), {DIST_BOLI_CENTRO}")
        mov.retroceder_distancia(DIST_BOLI_CENTRO)

        print("Bajando boli")
        dibujar.boli_bajar(prints=True)
      
      
    if opcion_menu == '0':
      dibujar.boli_bajar()
      
      
    elif opcion_menu == 'c':
      
      # ARRIBA
      print(f'\nGrados actuales arriba: {dibujar.grados_dibujar_arriba} grados.')
      input_dibujar_arriba_guardar = dibujar.grados_dibujar_arriba
      while True:
        input_dibujar_arriba = input("Introduce un nuevo ángulo para arriba (Enter para confirmar u omitir): ")
        if input_dibujar_arriba:
          dibujar.servo.herramienta_girar(float(input_dibujar_arriba), TIEMPO_TESTEOS)
          input_dibujar_arriba_guardar = input_dibujar_arriba
        else:
          if input_dibujar_arriba_guardar:
            dibujar.grados_dibujar_arriba = float(input_dibujar_arriba_guardar)
          break

      # ABAJO
      print(f'\nGrados actuales soltar: {dibujar.grados_dibujar_abajo} grados.')
      input_dibujar_abajo_guardar = dibujar.grados_dibujar_abajo
      while True:
        input_dibujar_abajo = input("Introduce un nuevo ángulo para abajo (Enter para confirmar u omitir): ")
        if input_dibujar_abajo:
          dibujar.servo.herramienta_girar(float(input_dibujar_abajo), TIEMPO_TESTEOS)
          input_dibujar_abajo_guardar = input_dibujar_abajo
        else:
          if input_dibujar_abajo_guardar:
            dibujar.grados_dibujar_abajo = float(input_dibujar_abajo_guardar)
          break

      
    elif opcion_menu == 'q':
      break
    
    else:
      print("Opcion no valida, 'q' para salir")

  mov.detener()
  rclpy.shutdown()

