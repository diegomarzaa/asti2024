import os
import sys
import rclpy
import subprocess
from time import sleep
from rclpy.node import Node
from geometry_msgs.msg import Twist
from final.Movements import Movements

class ButtonPublisher(Node):

    def __init__(self):
    
        super().__init__('button_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 300)
        self.publisher_
        
        self.counter = 0 # Diferenciar inicio de recepción paquetes
        self.tupla = (0.0, 0.0) # Tupla velocidades
        self.x = 0.0
        self.y = 0.0
        self.side_counter = False # Distinguir entre pulsar cruz por primera vez o tenerla mantenida
    
    def run_tcpdump(self, port):

        try:
            comando = ['sudo', 'tcpdump', '-i', 'wlo1', f'port {port}', '-X'] # Comando recibir paquetes
            proceso = subprocess.Popen(comando, stdout=subprocess.PIPE, universal_newlines=True) # Lanzar comando
            
            mov = Movements()

            for line in proceso.stdout: # Bucle recibir contenido paquete

                self.handle_line(line.strip(), mov) # Enviar contenido a método

            proceso.stdout.close()
            proceso.wait()

        except KeyboardInterrupt:
        
            self.get_logger().info("Proceso de tcpdump interrumpido.")

        finally:
        
            if proceso and proceso.poll() is None:

                proceso.kill()

    def handle_line(self, line, mov): # Lógica de código

        if line.startswith('0x0020:'): # Leemos contenido a partir de byte 0x0020

            bytes_str = line.split()[1]
            byte_of_interest = bytes_str[:4] # Bytes de interés son los dos siguientes
            
            if byte_of_interest == '0000' and self.counter == 0: # Print de primer paquete recibido
                print('\n\n░░░░░░░░░░░░░░░░░░░░░░█████████░░░░░░░░░')
                print('░░███████░░░░░░░░░░███▒▒▒▒▒▒▒▒███░░░░░░░')
                print('░░█▒▒▒▒▒▒█░░░░░░░███▒▒▒▒▒▒▒▒▒▒▒▒▒███░░░░')
                print('░░░█▒▒▒▒▒▒█░░░░██▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒██░░')
                print('░░░░█▒▒▒▒▒█░░░██▒▒▒▒▒██▒▒▒▒▒▒██▒▒▒▒▒███░')
                print('░░░░░█▒▒▒█░░░█▒▒▒▒▒▒████▒▒▒▒████▒▒▒▒▒▒██')
                print('░░░█████████████▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒██')
                print('░░░█▒▒▒▒▒▒▒▒▒▒▒▒█▒▒▒▒▒▒▒▒▒█▒▒▒▒▒▒▒▒▒▒▒██')
                print('░██▒▒▒▒▒▒▒▒▒▒▒▒▒█▒▒▒██▒▒▒▒▒▒▒▒▒▒██▒▒▒▒██')
                print('██▒▒▒███████████▒▒▒▒▒██▒▒▒▒▒▒▒▒██▒▒▒▒▒██')
                print('█▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒█▒▒▒▒▒▒████████▒▒▒▒▒▒▒██')
                print('██▒▒▒▒▒▒▒▒▒▒▒▒▒▒█▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒██░')
                print('░█▒▒▒███████████▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒██░░░')
                print('░██▒▒▒▒▒▒▒▒▒▒████▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒█░░░░░')
                print('░░████████████░░░█████████████████░░░░░░')
                print('\n##########')
                print('# START! #')
                print('##########\n')
                self.counter += 1
                    
            elif byte_of_interest == '0000' and self.counter >= 1: # Si no pulsamos ningún botón, self.side_counter -> False, haciendo que podamos volver a usar la cruceta
                self.side_counter = False
            
            elif byte_of_interest == '0004': # X, para detener el robot
                self.tupla = (0.0, 0.0)
                self.x = 0.0
                self.y = 0.0
                msg = Twist()
                msg.linear.x = self.tupla[0]
                msg.angular.z = self.tupla[1]
                self.publisher_.publish(msg)

            elif byte_of_interest == '4000' and self.x <= 2.0 and self.side_counter == False: # UP, aumentar velocidad linear cada vez que pulsemos cruz hacia arriba
                self.side_counter = True
                self.x += 0.1
                self.tupla = (self.x, self.y)
                msg = Twist()
                msg.linear.x = self.tupla[0]
                msg.angular.z = self.tupla[1]
                self.publisher_.publish(msg)
                    
            elif byte_of_interest == '8000' and self.x >= -2.0 and self.side_counter == False: # DOWN, disminuir velocidad linear cada vez que pulsemos cruz hacia abajo
                self.side_counter = True
                self.x -= 0.1
                self.tupla = (self.x, self.y)
                msg = Twist()
                msg.linear.x = self.tupla[0]
                msg.angular.z = self.tupla[1]
                self.publisher_.publish(msg)
                   
            elif byte_of_interest == '2000' and self.y <= 2.0 and self.side_counter == False: # LEFT, aumentar velocidad angular izquierda cada vez que pulsemos cruz hacia izquierda
                self.side_counter = True
                self.y += 0.1
                self.tupla = (self.x, self.y)
                msg = Twist()
                msg.linear.x = self.tupla[0]
                msg.angular.z = self.tupla[1]
                self.publisher_.publish(msg)
                    
            elif byte_of_interest == '1000' and self.y >= -2.0 and self.side_counter == False: # RIGHT, aumentar velocidad angular derecha cada vez que pulsemos cruz hacia derecha
                self.side_counter = True
                self.y -= 0.1
                self.tupla = (self.x, self.y)
                msg = Twist()
                msg.linear.x = self.tupla[0]
                msg.angular.z = self.tupla[1]
                self.publisher_.publish(msg)
            
            elif byte_of_interest == '0001': # R, sube pale al pulsar botón R
                mov.pale_subir()
                
            elif byte_of_interest == '0002': # L, baja pale al pulsar botón L
                mov.pale_bajar()
            
            elif byte_of_interest == '0800': # START
                diego_putero # Detiene ejecución

def main(args=None):
    
    rclpy.init(args=args)
    button_publisher = ButtonPublisher()
    button_publisher.run_tcpdump('8888')
    rclpy.spin(button_publisher)
    button_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':

    main()

