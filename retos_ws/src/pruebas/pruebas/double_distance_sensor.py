#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO_TRIGGERder = 23
GPIO_ECHOder = 24
GPIO_TRIGGERizq = 20
GPIO_ECHOizq = 21



GPIO.setup(GPIO_TRIGGERder, GPIO.OUT)
GPIO.setup(GPIO_ECHOder, GPIO.IN)
GPIO.setup(GPIO_TRIGGERizq, GPIO.OUT)
GPIO.setup(GPIO_ECHOizq, GPIO.IN)

class DistanceSensorPublisher(Node):
    
        def __init__(self):
            super().__init__('distance_sensor_publisher')
            self.publisher_der = self.create_publisher(Float32, 'distance_der', 10)
            self.publisher_izq = self.create_publisher(Float32, 'distance_izq', 10)
            timer_period = 0.5  # seconds
            self.timer = self.create_timer(timer_period, self.timer_callback)
    
        def timer_callback(self):
            distance_der = self.distance(GPIO_TRIGGERder, GPIO_ECHOder)
            distance_izq = self.distance(GPIO_TRIGGERizq, GPIO_ECHOizq)
            msg_der = Float32()
            msg_der.data = distance_der
            self.publisher_der.publish(msg_der)
            msg_izq = Float32()
            msg_izq.data = distance_izq
            self.publisher_izq.publish(msg_izq)
    
        def distance(self, TRIG, ECHO):
            GPIO.output(TRIG, True)
            time.sleep(0.00001)
            GPIO.output(TRIG, False)
            start_time = time.time()
            stop_time = time.time()
            while GPIO.input(ECHO) == 0:
                start_time = time.time()
            while GPIO.input(ECHO) == 1:
                stop_time = time.time()
            time_elapsed = stop_time - start_time
            distance = (time_elapsed * 34300) / 2
            return distance

def main(args=None):
    rclpy.init(args=args)
    distance_sensor_publisher = DistanceSensorPublisher()
    rclpy.spin(distance_sensor_publisher)
    distance_sensor_publisher.destroy_node()
    rclpy.shutdown()
    GPIO.cleanup()

if __name__ == '__main__':
    main()