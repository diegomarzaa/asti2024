import rclpy
from rclpy.node import Node
from final.Movements import Movements
from final.Movements import Servo

# TODO: Configuración de velocidades, poder cambiar entre rondas para así ajustarlo a la mejor puntuación posible.

class Bolos(Node):
  def __init__(self):
    super().__init__('bolos')
    
    self.mov = Movements()
    self.distancia_inicial = 0.40
    self.grados_izquierda = 95
    self.radio_izquierda = 0.2
    self.distancia_final = 0.25
    
    self.vel_inicial = 0.10
    self.vel_lanzamiento = 0.2
    
    self.servo = Servo()
    
    self.grados_bolos_mantener = 0.0
    self.grados_bolos_soltar = 50.0
    self.tiempo_bolos_mantener = 0.2
    self.tiempo_bolos_soltar = 0.2          # TODO: MODIFICAR ESTO, BAJAR NUMERO
    

  def bolos_soltar(self):
    self.servo.herramienta_girar(self.grados_bolos_soltar, self.tiempo_bolos_soltar)
    
  def bolos_mantener(self):
    self.servo.herramienta_girar(self.grados_bolos_mantener, self.tiempo_bolos_mantener)
    
  def actualizar_grados_bolos(self, grados_mantener, grados_soltar):
    self.grados_bolos_mantener = grados_mantener
    self.grados_bolos_soltar = grados_soltar
    
  def actualizar_tiempo_bolos(self, tiempo_mantener, tiempo_soltar):
    self.tiempo_bolos_mantener = tiempo_mantener
    self.tiempo_bolos_soltar = tiempo_soltar


def main(args=None):
  rclpy.init(args=args)
  bolos = Bolos()
  
  while True:
    input("\n\nEnter: Posicionar servo en posición inicial")
    bolos.bolos_mantener()
    
    opcion_menu = input("\nEnter: Iniciar lanzamiento de bolos\nc: Calibrar movimiento\nh: Calibrar herramienta grados\nj: Calibrar herramienta tiempos\nq: Salir\n")
    if opcion_menu == 'q':
      break
    
    if opcion_menu == 'h':
      TIEMPO_TESTEOS = 2
      # MANTENER
      print(f'\nGrados actuales mantener: {bolos.grados_bolos_mantener} grados.')
      input_mantener_guardar = bolos.grados_bolos_mantener
      while True:
        input_mantener = input("Introduce un nuevo ángulo para mantener (Enter para confirmar u omitir): ")
        if input_mantener:
          bolos.servo.herramienta_girar(float(input_mantener), TIEMPO_TESTEOS)
          input_mantener_guardar = input_mantener
        else:
          if input_mantener_guardar:
            bolos.grados_bolos_mantener = float(input_mantener_guardar)
          break

      print(f'\nGrados actuales soltar: {bolos.grados_bolos_soltar} grados.')
      input_soltar_guardar = bolos.grados_bolos_soltar
      while True:
        input_soltar = input("Introduce un nuevo ángulo para soltar (Enter para confirmar u omitir): ")
        if input_soltar:
          bolos.servo.herramienta_girar(float(input_soltar), TIEMPO_TESTEOS)
          input_soltar_guardar = input_soltar
        else:
          if input_soltar_guardar:
            bolos.grados_bolos_soltar = float(input_soltar_guardar)
          break
      continue
          
    if opcion_menu == 'j':
      input_mantener = input(f"\nTiempo actual mantener: {bolos.tiempo_bolos_mantener} s.\nNuevo tiempo mantener (Enter para omitir): ")
      if input_mantener:
        bolos.tiempo_bolos_mantener = input_mantener
      
      input_soltar = input(f"\nTiempo actual soltar: {bolos.tiempo_bolos_soltar} s.\nNuevo tiempo soltar (Enter para omitir): ")
      if input_soltar:
        bolos.tiempo_bolos_soltar = input_soltar
      continue

        
    if opcion_menu == 'c':
      distancia_inicial_calib = input(f"\nDistancia inicial actual: {bolos.distancia_inicial} m.\nNueva distancia inicial (Enter para omitir): ")
      if distancia_inicial_calib:
        bolos.distancia_inicial = float(distancia_inicial_calib)
        
      grados_izquierda_calib = input(f"\nGrados izquierda actual: {bolos.grados_izquierda} grados.\nNuevos grados izquierda (Enter para omitir): ")
      if grados_izquierda_calib:
        bolos.grados_izquierda = float(grados_izquierda_calib)
      
      radio_izquierda_calib = input(f"\nRadio izquierda actual: {bolos.radio_izquierda} m.\nNuevo radio izquierda (Enter para omitir): ")
      if radio_izquierda_calib:
        bolos.radio_izquierda = float(radio_izquierda_calib)
      
      distancia_final_calib = input(f"\nDistancia final actual: {bolos.distancia_final} m.\nNueva distancia final (Enter para omitir): ")
      if distancia_final_calib:
        bolos.distancia_final = float(distancia_final_calib)
      
      continue



    # EJECUCIÓN
    print("Posicionando el robot para lanzar los bolos marcha atrás")
    bolos.mov.actualizar_vel_lineal(bolos.vel_inicial)
    bolos.mov.retroceder_distancia(bolos.distancia_inicial, aceleracion=False)
    bolos.mov.girar_grados_der_atras(bolos.grados_izquierda, radio=bolos.radio_izquierda)        # Derecha realmente es izquierda si fuera hacia adelante
    
    # Opción hacia delante (no usar)
    # print("Posicionando el robot para lanzar los bolos palante")
    # self.mov.avanzar_distancia(self.distancia_inicial)   # Avanzar un poquito para no tocar la pared en el siguiente paso
    # self.mov.girar_grados_izq(self.grados_izquierda, radio=self.radio_izquierda)    # Girar y ya avanzar para ponerse en la posición correcta.
    # self.mov.avanzar_distancia(self.distancia_final)     # Ponerse un poquito más cerca de la linea, asegurarse de no tocarla.

    print("Lanzando los bolos mientras retrocede")
    bolos.mov.actualizar_vel_lineal(bolos.vel_lanzamiento)
    bolos.mov.retroceder_distancia(bolos.distancia_final/2, aceleracion=False)
    bolos.bolos_soltar()
    bolos.mov.retroceder_distancia(bolos.distancia_final/2, aceleracion=False)
    
    
  rclpy.shutdown()
