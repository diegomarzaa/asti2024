# FICHERO DESACTUALIZADO, VER EL FICHERO MOVIMIENTOS.PY EN LA CARPETA DE LA FINAL

import time
import math

# -------- FUNCIONES AUXILIARES ----------------

def avanzar(pub, vel=0.3):
    pub.publish_velocity((vel, 0.0))

def retroceder(pub, vel=0.3):
    pub.publish_velocity((-vel, 0.0))
    
def detener(pub):
    pub.publish_velocity((0.0, 0.0))

def girar_izquierda(pub, vel=2.0):
    pub.publish_velocity((0.0, vel))
    
def girar_derecha(pub, vel=2.0):
    pub.publish_velocity((0.0, -vel))


# -------- FUNCIONES POR DISTANCIA ----------------

def recto(pub, max_vel, distancia_total, acc_lin=0.005):
    # HECHO
    # TODO: Asegurarse que si la aceleracion es muy grande, no se pase de la velocidad maxima

    vel_linear = 0.01
    while(distancia_total >= 0):
        if (vel_linear < max_vel):
            vel_linear += acc_lin
        pub.publish_velocity((vel_linear, 0.0))
        time.sleep(0.1)
        distancia_recorrida = vel_linear*0.1
        distancia_total -= distancia_recorrida

    pub.publish_velocity((0.0, 0.0))

def atras(pub, max_vel, distancia_total, acc_lin=0.005):
    # HECHO
    
    vel_linear = 0.01
    while(distancia_total >= 0):
        if (vel_linear < max_vel):
            vel_linear += acc_lin
        pub.publish_velocity((-vel_linear, 0.0))
        time.sleep(0.1)
        distancia_recorrida = vel_linear*0.1
        distancia_total -= distancia_recorrida

    pub.publish_velocity((0.0, 0.0))

def derecha(pub, max_vel, degrees, acc_ang=0.01):
    #HECHO
    
    grados_total = degrees*math.pi/180
    
    vel_angular = 0.01
    while(grados_total >= 0):
        if (vel_angular < max_vel):
            vel_angular += acc_ang
        pub.publish_velocity((0.0, -vel_angular))
        time.sleep(0.1)
        grados_recorridos = vel_angular*0.1
        grados_total -= grados_recorridos

    pub.publish_velocity((0.0, 0.0))

def izquierda(pub, max_vel, degrees, acc_ang=0.01):
    # HECHO
    
    grados_total = degrees*math.pi/180
    
    vel_angular = 0.01
    while(grados_total >= 0):
        if (vel_angular < max_vel):
            vel_angular += acc_ang
        pub.publish_velocity((0.0, vel_angular))
        time.sleep(0.1)
        grados_recorridos = vel_angular*0.1
        grados_total -= grados_recorridos

    pub.publish_velocity((0.0, 0.0))


# -------- FUNCIONES ROTACION CON RADIO ----------------

def avanzar_derecha(pub, vel_lin, radio, grados):
    # TODO: Comprobar el robot físico
    
    vel_lin -= 0.1
    radian_total = grados*math.pi/180
    
    while(radian_total >= 0.2):          # TODO: ver vien
        print(vel_lin/radio)
        angular_vel:float = vel_lin/radio
        
        pub.publish_velocity((vel_lin, -angular_vel))
        
        time.sleep(0.1)
        radian_recorridos = angular_vel * 0.1             # pi/3 * 0.1 = pi/30
        radian_total -= radian_recorridos

    pub.publish_velocity((0.0, 0.0))
    
def avanzar_izquierda(pub, vel_lin, radio, grados):
    # TODO: Comprobar el robot físico
    
    vel_lin -= 0.1
    radian_total = grados*math.pi/180
    
    while(radian_total >= 0.2):
        
            
        angular_vel = vel_lin/radio
        
        pub.publish_velocity((vel_lin, angular_vel))
        
        time.sleep(0.1)
        radian_recorridos = angular_vel * 0.1             # pi/3 * 0.1 = pi/30
        radian_total -= radian_recorridos

    pub.publish_velocity((0.0, 0.0))