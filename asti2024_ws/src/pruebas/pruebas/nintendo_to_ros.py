from time import sleep
import subprocess
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import sys
import os
import signal
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped

class Button_publisher(Node):

    def __init__(self):
        super().__init__('button_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 300)
        self.publisher_camera_ = self.create_publisher(JointTrajectory, '/set_joint_trajectory', 10)
        self.publisher_camera_
        self.publisher_
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        
        self.tupla = (0.0, 0.0)
        self.x = 0.0
        self.y = 0.0
        self.camera_tilt_angle = 0.0
        self.max_camera_tilt = 4.0
        self.min_camera_tilt = -4.0
        self.counter = 0
        
        self.side_counter = False
        
        self.port = '8888'
        
        self.count = 0
        
    def timer_callback(self):
        self.run()
    
    def publish_camera_tilt_angle(self, angle):
        msg = JointTrajectory()
        msg.header = Header()
        msg.header.frame_id = 'world'
        msg.joint_names = ['camera_joint']
        point = JointTrajectoryPoint()
        point.positions = [0.0]
        msg.points = [point]
        self.publisher_camera_.publish(msg)
        self.get_logger().info('Published message')
        
    def publish_camera_tilt_angle2(self, angle):        
        msg = JointTrajectory()
        msg.header = Header()
        msg.header.frame_id = 'world'
        msg.joint_names = ['camera_joint']
        point = JointTrajectoryPoint()
        point.positions = [4.0]
        msg.points = [point]
        self.publisher_camera_.publish(msg)
        self.get_logger().info('Published message')
		
    def run(self):
        # Ejecutar el comando tcpdump en la terminal
        print('\nListening on port', self.port)
        print('Please be patient, this might take a while\n')
        print('╦╦╦╦╦╦▄▀▀▀▀▀▀▄╦╦╦╦╦╦')
        print('▒▓▒▓▒█╗░░▐░░░╔█▒▓▒▓▒')
        print('▒▓▒▓▒█║░░▐▄▄░║█▒▓▒▓▒')
        print('▒▓▒▓▒█╝░░░░░░╚█▒▓▒▓▒')
        print('╩╩╩╩╩╩▀▄▄▄▄▄▄▀╩╩╩╩╩╩\n')
        comando0 = ['ros2', 'run', 'pruebas', 'test_vision_gazebo']
        proceso0 = subprocess.Popen(comando0, preexec_fn=os.setsid) 
        comando = ['sudo', 'tcpdump', '-i', 'wlo1', 'port', self.port, '-X']
        proceso = subprocess.Popen(comando, stdout=subprocess.PIPE)    
        print(proceso)  
        try:
            # Leer la salida línea por línea
            for linea in proceso.stdout:
                # Decodificar la línea a UTF-8
                linea_decodificada = linea.decode('utf-8').strip()
                linea = linea_decodificada.split(' ')
                columna = linea[2]
                if columna == '0000' and self.counter == 0:
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
                elif columna == '0000' and self.counter >= 1:
                    
                    self.side_counter = False
                    
                elif columna == '0100': # A
                    self.tupla = (0.2, 0.0)
                    self.x = 0.2
                    msg = Twist()
                    msg.linear.x = self.tupla[0]
                    msg.angular.z = self.tupla[1]
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"({msg.linear.x}, {msg.angular.z})")
                elif columna == '0200': # B
                    self.tupla = (-0.2, 0.0)
                    self.x = -0.2
                    msg = Twist()
                    msg.linear.x = self.tupla[0]
                    msg.angular.z = self.tupla[1]
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"({msg.linear.x}, {msg.angular.z})")
                elif columna == '0004': # X
                    self.tupla = (0.0, 0.0)
                    self.x = 0.0
                    self.y = 0.0
                    msg = Twist()
                    msg.linear.x = self.tupla[0]
                    msg.angular.z = self.tupla[1]
                    
                    msg.linear.y = 0.0
                    msg.linear.z = 0.0
                    msg.angular.x = 0.0
                    msg.angular.y = 0.0
                    
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"({msg.linear.x}, {msg.linear.y}, {msg.linear.z},{msg.angular.x}, {msg.angular.z},{msg.angular.z})")
                elif columna == '0008': # Y
                    msg = Twist()
                    msg.linear.x = 2.0
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"{msg.linear.x}")
                elif columna == '4000' and self.x <= 2.0 and self.side_counter == False: # UP
                    self.side_counter = True
                    self.y = 0.0
                    self.x += 0.2
                    self.tupla = (self.x, 0.0)
                    msg = Twist()
                    msg.linear.x = self.tupla[0]
                    msg.angular.z = self.tupla[1]
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"({msg.linear.x}, {msg.angular.z})")
                    
                elif columna == '8000' and self.x >= -2.0 and self.side_counter == False: # DOWN
                    self.side_counter = True
                    self.y = 0.0
                    self.x -= 0.2
                    self.tupla = (self.x, 0.0)
                    msg = Twist()
                    msg.linear.x = self.tupla[0]
                    msg.angular.z = self.tupla[1]
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"({msg.linear.x}, {msg.angular.z})")
                    
                elif columna == '2000' and self.y <= 2.0 and self.side_counter == False: # LEFT
                    self.side_counter = True
                    self.x = 0.0
                    self.y += 0.3
                    self.tupla = (0.0, self.y)
                    msg = Twist()
                    msg.linear.x = self.tupla[0]
                    msg.angular.z = self.tupla[1]
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"({msg.linear.x}, {msg.angular.z})")
                    
                elif columna == '1000' and self.y >= -2.0 and self.side_counter == False: # RIGHT
                    self.side_counter = True
                    self.x = 0.0
                    self.y -= 0.3
                    self.tupla = (0.0, self.y)
                    msg = Twist()
                    msg.linear.x = self.tupla[0]
                    msg.angular.z = self.tupla[1]
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"({msg.linear.x}, {msg.angular.z})")
                    
                elif columna == '0001': # R
                    
                    self.camera_tilt_angle += 0.1
                    self.publish_camera_tilt_angle(self.camera_tilt_angle)
                        
                elif columna == '0002': # L
                    self.camera_tilt_angle -= 0.1
                    self.publish_camera_tilt_angle2(self.camera_tilt_angle)

                elif columna == '0800': # START
                    print('Shutting down...')
                    os.killpg(os.getpgid(proceso0.pid), signal.SIGTERM)
                    exit()
            
                
        except KeyboardInterrupt:
            proceso.terminate()

def main(args=None):
    rclpy.init(args=args)
    button_pub = Button_publisher()
    rclpy.spin(button_pub)
    button_pub.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

