import rclpy
from rclpy.node import Node
from final.Movements import Movements

# TODO: Configuración de velocidades, poder cambiar entre rondas para así ajustarlo a la mejor puntuación posible.

# Comprobar valores

distancia_aparcamiento = 0.75
grados_aparcamiento = 90
radio_aparcamiento = 0.35
distancia_pale = 0.35 # Reducir velocidad y mejorar el control a esa distancia del pale
vel_a_pale = 0.1      # Velocidad de aproximación al pale
vel_defecto = mov.obj_linear_vel    # Si no funciona, función de Movements
pale = False

def avanzar(mov, color):
    
    # Implementar codigo de siguelineas en bucle, que busque el color
    distancia = 3   # Actualizar con vision?
    # Se asume que esta sobre la linea guia, enderezadoal pale
    if distancia <= distancia_pale and not pale:
        recoger_pale(mov)
    elif distancia <= distancia_pale and pale:
        dejar_pale(mov)
        return 1

def aparcar(mov):
    
    # Uso de sensores?
    
    mov.avanzar_distancia(distancia_aparcamiento)
    mov.girar_grados(grados_aparcamiento, radio_aparcamiento, False) # Gira y se posiciona para la marcha atras
    mov.avanzar_distancia(-0.2) # Ultima marcha atras
    
def recoger_pale(mov):      # Cuando esta cerca del pale
    
    mov.pale_bajar()
    mov.actualizar_vel_lineal(vel_a_pale)
    mov.avanzar_distancia(distancia_pale)   # Mejor con visión
    mov.pale_subir()
    mov.actualizar_vel_lineal(vel_defecto)   
    pale = True
    
def abrir_puerta(mov):
    
    # Implementar
    pass

def dejar_pale(mov):
    
    mov.avanzar_distancia(distancia_pale)
    mov.pale_bajar()
    mov.avanzar_distancia(-0.5)
    mov.girar_grados(180,True)
    mov.pale_subir()
    pale = False

def avanzar_tarjeta(mov):
    
    # No se donde va la tarjeta
    pass

def reconocer_tarjeta(mov):
    
    # Implementar código de reconocer figuras
    pass

def main(args=None):
  global distancia_inicial, grados_izquierda, radio_izquierda, distancia_final
  
  rclpy.init(args=args)
  node = rclpy.create_node('Fabrica')
  mov = Movements()
  
  while True:
    opcion_menu = input("Enter para iniciar la minifábrica (q para salir): ")
    if opcion_menu == 'q':
      break
    for i in range(4):
        avanzar_tarjeta(mov)
        color = reconocer_tarjeta(mov)
        avanzar(mov, color)
    aparcar(mov)
    
    distancia_inicial_calib = input(f"\nDistancia inicial actual: {distancia_inicial} m.\nNueva distancia inicial (Enter para omitir): ")
    if distancia_inicial_calib:
      distancia_inicial = float(distancia_inicial_calib)
      
    grados_izquierda_calib = input(f"\nGrados izquierda actual: {grados_izquierda} grados.\nNuevos grados izquierda (Enter para omitir): ")
    if grados_izquierda_calib:
      grados_izquierda = float(grados_izquierda_calib)
    
    radio_izquierda_calib = input(f"\nRadio izquierda actual: {radio_izquierda} m.\nNuevo radio izquierda (Enter para omitir): ")
    if radio_izquierda_calib:
      radio_izquierda = float(radio_izquierda_calib)
    
    distancia_final_calib = input(f"\nDistancia final actual: {distancia_final} m.\nNueva distancia final (Enter para omitir): ")
    if distancia_final_calib:
      distancia_final = float(distancia_final_calib)
    
  rclpy.shutdown()  
  
    
