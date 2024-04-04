import rclpy
from rclpy.node import Node
from final.Movements import Movements

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
    
  def posicionarse_patras(self):
    # TODO: Testear
    # Este es el que rentaría, si la rampa está delante del robot, la cosa sería empezar con el robot al revés.
    print("Posicionando el robot para lanzar los bolos patras")
    self.mov.actualizar_vel_lineal(self.vel_inicial)
    self.mov.retroceder_distancia(self.distancia_inicial, aceleracion=False)
    self.mov.girar_grados_der_atras(self.grados_izquierda, radio=self.radio_izquierda)        # Derecha realmente es izquierda si fuera hacia adelante

  def posicionarse_palante(self):
    print("Posicionando el robot para lanzar los bolos palante")
    self.mov.avanzar_distancia(self.distancia_inicial)   # Avanzar un poquito para no tocar la pared en el siguiente paso
    self.mov.girar_grados_izq(self.grados_izquierda, radio=self.radio_izquierda)    # Girar y ya avanzar para ponerse en la posición correcta.
    self.mov.avanzar_distancia(self.distancia_final)     # Ponerse un poquito más cerca de la linea, asegurarse de no tocarla.
    
  def posicionarse_palante_patras(self):
    # (Modo aparcamiento)
    print("Posicionando el robot para lanzar los bolos palante y patras")
    self.mov.avanzar_distancia(self.distancia_inicial*3)  # Hay que pasarse para luego recular
    self.mov.girar_grados__atras(self.grados_izquierda, radio=self.radio_izquierda)
    self.mov.retroceder_distancia(self.distancia_final)
    
  def lanzar(self):
    print("Lanzando los bolos")
    self.mov.bolos_soltar()
    
  def lanzar_mientras_retrocede(self):
    print("Lanzando los bolos mientras retrocede")
    self.mov.actualizar_vel_lineal(self.vel_lanzamiento)
    self.mov.bolos_soltar()
    self.mov.retroceder_distancia(self.distancia_final, aceleracion=False)    # Chetar la velocidad aquí para que dé impulso


def main(args=None):
  rclpy.init(args=args)
  bolos = Bolos()
  
  while True:
    opcion_menu = input("Enter para iniciar el lanzamiento de bolos (q para salir): ")
    if opcion_menu == 'q':
      break
    
    bolos.posicionarse_patras()
    bolos.lanzar_mientras_retrocede()
    
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
    
  rclpy.shutdown()
