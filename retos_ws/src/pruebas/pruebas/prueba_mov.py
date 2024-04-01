import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import atan2, pow, sqrt, pi
import numpy as np
import time

global_x = 0.0
global_y = 0.0
angle_x = 0.0
angle_y = 0.0
angle_z = 0.0
angle_w = 0.0
initial_x = 0.0
initial_y = 0.0
initial_yaw = 0.0
yaw = 0.0
stop = False
distance_x = 0.0
distance_y = 0.0
angle_difference = 0.0

def odom_callback(msg):
    global global_x, global_y, angle_x, angle_y, angle_z, angle_w, yaw, initial_x, initial_y, initial_yaw, distance_x, distance_y, angle_difference
    global_x = msg.pose.pose.position.x
    global_y = msg.pose.pose.position.y  
    angle_x = msg.pose.pose.orientation.x
    angle_y = msg.pose.pose.orientation.y
    angle_z = msg.pose.pose.orientation.z
    angle_w = msg.pose.pose.orientation.w

    yaw = atan2(2*(angle_w*angle_z + angle_x*angle_y), 1 - 2*(angle_y*angle_y + angle_z*angle_z))

    print(f"Current position: ({global_x}, {global_y})")
    print(f"Current yaw angle: {yaw}")
    
    if global_x == 0 and global_y == 0 and yaw == 0:
        initial_x = global_x
        initial_y = global_y
        initial_yaw = yaw
        print(f"Initial position and angle: ({initial_x}, {initial_y}, {initial_yaw})")
        
    distance_x = global_x - initial_x
    distance_y = global_y - initial_y
    print(f"Distance between positions: ({distance_x}, {distance_y})")
    
    angle_difference = abs(yaw - initial_yaw)
    print(f"Angle difference: {angle_difference}")

def main(args=None): # Aún está hecho como una mierda y al volver no se acaba de centrar (esa parte solo tiene función en el simulador para evitar tener que mover el robot manualmente
                     # Pero en la vida real no hace falta, así que lo que necesitamos ya lo hace. Gira y va al destino.
                     # Tendríamos que cambiar el nombre de main() a la figura que sea, y poner como parametros la distancia entre los puntos y el angulo deseado
                     # Trataré de limpiarlo un poco xd
    rclpy.init(args=args)
    node = rclpy.create_node('square')
    publisher = node.create_publisher(Twist, 'cmd_vel', 10)
    subscription = node.create_subscription(Odometry, 'odom', odom_callback, 10)
    
    message = Twist()

    linear_speed = 0.1
    angular_speed = -(pi / 20)
    square_length = 0.9

    
    global initial_x, initial_y, initial_yaw, stop
    initial_x = global_x
    initial_y = global_y
    initial_yaw = yaw
       
    while rclpy.ok() and not stop:
        if angle_difference <= 0.43:
            print('Looking for coordinate')
            message.angular.z = angular_speed
            publisher.publish(message)
            rclpy.spin_once(node)
            time.sleep(0.1)  # sleeps for 100 milliseconds
        else:
            print('Coordinate found')
            message.linear.x = 0.0
            message.angular.z = 0.0
            publisher.publish(message)
            stop = True
              
    stop = False

    while rclpy.ok() and not stop:
        if sqrt(pow(distance_x, 2) + pow(distance_y, 2)) <= square_length:
            print('Going to coordinate')
            message.linear.x = linear_speed
            publisher.publish(message)
            rclpy.spin_once(node)
            time.sleep(0.1)  # sleeps for 100 milliseconds
        else:
            print('Coordinate found')
            message.linear.x = 0.0
            publisher.publish(message)
            stop = True
    print('exiting')
    initial_x = global_x
    initial_y = global_y
    stop = False

        
    initial_yaw = yaw
    message.linear.x = 0.0
    message.angular.z = 0.0
    publisher.publish(message)
    time.sleep(2)  # sleeps for 100 milliseconds
    
    while rclpy.ok() and not stop:
        if angle_difference <= 3.13:
            print('Looking for coordinate')
            message.angular.z = -angular_speed
            publisher.publish(message)
            rclpy.spin_once(node)
            time.sleep(0.1)  # sleeps for 100 milliseconds
        else:
            print('Coordinate found')
            message.linear.x = 0.0
            message.angular.z = 0.0
            publisher.publish(message)
            stop = True
              
    stop = False
    
    while rclpy.ok() and not stop:
        if sqrt(pow(distance_x, 2) + pow(distance_y, 2)) <= square_length:
            print('Going to coordinate')
            message.linear.x = linear_speed
            publisher.publish(message)
            rclpy.spin_once(node)
            time.sleep(0.1)  # sleeps for 100 milliseconds
        else:
            print('Coordinate found')
            message.linear.x = 0.0
            publisher.publish(message)
            stop = True
    
    print('exiting')
    initial_x = global_x
    initial_y = global_y
    stop = False

        
    initial_yaw = yaw
    message.linear.x = 0.0
    message.angular.z = 0.0
    publisher.publish(message)
    time.sleep(0.1)  # sleeps for 100 milliseconds
    
    while rclpy.ok() and not stop:
        if angle_difference <= pi - 0.43:
            print('Looking for coordinate')
            message.angular.z = angular_speed
            publisher.publish(message)
            rclpy.spin_once(node)
            time.sleep(0.1)  # sleeps for 100 milliseconds
        else:
            print('Coordinate found')
            print('Ulti')
            message.linear.x = 0.0
            message.angular.z = 0.0
            publisher.publish(message)
            stop = True
    initial_x = global_x
    initial_y = global_y          
    stop = False
    initial_yaw = yaw
    message.linear.x = 0.0
    message.angular.z = 0.0
    publisher.publish(message)
    time.sleep(0.1)

    rclpy.spin_once(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
