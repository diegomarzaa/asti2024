#!/usr/bin/env python3

# Import the necessary ROS2 libraries
import rclpy
from time import sleep
from math import pi, atan
from final.Movements import Movements
from final.Sensors import Sensors

def run(mov, sensors):
    mov.actualizar_vel_lineal(0.2)
    mov.actualizar_vel_angular(0.2)
    while True:
        # sensores.distancias() -> [izq, izq_f, der_f, der]
        distancias = sensors.distancias()
        while distancias[3] == 5.0:
            mov.avanzar()

        if distancias[3] > 5.0:
            mov.publish_wheel_velocity(0.2, -0.3)

        elif distancias[3] < 5.0:
            mov.publish_wheel_velocity(0.2, 0.3)

        elif distancias[2] < 15.0 and distancias[1] < 15.0:
            while distancias[2] < distancias[1]:
                mov.girar_izquierda()

        elif distancias[2] < 5.0 or distancias[1] < 5.0 or (distancias[2] < 5.0 and distancias[1] < 5.0):
            while distancias[1] < 5.0 and distancias[2] < 5.0:
                mov.girar_derecha()
                
        elif distancias[0] < 5.0:
            while distancias[0] < 5.0:
                mov.girar_derecha()
        sleep(0.1)

def main(args=None):
    rclpy.init(args=None)
    node = rclpy.create_node('laberinto')
    mov = Movements()
    sensors = Sensors()
    run(mov, sensors)

if __name__ == '__main__':
    main()
