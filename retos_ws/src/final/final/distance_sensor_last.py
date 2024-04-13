#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO_TRIGGERder = 23
GPIO_ECHOder = 24
GPIO_TRIGGERizq = 20
GPIO_ECHOizq = 21
GPIO_TRIGGERdelante_der = 27
GPIO_ECHOdelante_der = 22
GPIO_TRIGGERdelante_izq = 5
GPIO_ECHOdelante_izq = 6


GPIO.setup(GPIO_TRIGGERder, GPIO.OUT)
GPIO.setup(GPIO_ECHOder, GPIO.IN)

GPIO.setup(GPIO_TRIGGERizq, GPIO.OUT)
GPIO.setup(GPIO_ECHOizq, GPIO.IN)

GPIO.setup(GPIO_TRIGGERdelante_izq, GPIO.OUT)
GPIO.setup(GPIO_ECHOdelante_izq, GPIO.IN)
GPIO.setup(GPIO_TRIGGERdelante_der, GPIO.OUT)
GPIO.setup(GPIO_ECHOdelante_der, GPIO.IN)


class DistanceSensorPublisher(Node):
    
        def __init__(self):
            super().__init__('distance_sensor_publisher')
            self.publisher_distancias = self.create_publisher(Float32MultiArray, '/distancias_sensores', 200)
            
            timer_period = 0.2  # seconds
            self.timer = self.create_timer(timer_period, self.timer_callback)
    
        def timer_callback(self):
            distance_der = self.distance(GPIO_TRIGGERder, GPIO_ECHOder)
            distance_izq = self.distance(GPIO_TRIGGERizq, GPIO_ECHOizq)
            distance_delante_der = self.distance(GPIO_TRIGGERdelante_der, GPIO_ECHOdelante_der)
            distance_delante_izq = self.distance(GPIO_TRIGGERdelante_izq, GPIO_ECHOdelante_izq)
            
            # DER
            if distance_der is None:
                distance_der = 0.0
            
            # IZQ
            if distance_izq is None:
                distance_izq = 0.0
            
            # DELANTE IZQ
            if distance_delante_izq is None:
                distance_delante_izq = 0.0
            
            # DELANTE DER
            if distance_delante_der is None:
                distance_delante_der = 0.0

            msg = Float32MultiArray()
            msg.data = [distance_izq, distance_delante_izq, distance_delante_der, distance_der]
            self.publisher_distancias.publish(msg)
            self.get_logger().info('Float32MultiArray (izq, delante_izq, delante_der, der): ' + str(msg.data))
    
        def distance(self, TRIG, ECHO):
            GPIO.output(TRIG, True)
            time.sleep(0.00001)
            GPIO.output(TRIG, False)
            start_time = time.time()
            stop_time = time.time()
            timeout = 0.04
            
            while GPIO.input(ECHO) == 0:
                start_time = time.time()
                if start_time - stop_time > timeout:
                    return 0.0
                    
            while GPIO.input(ECHO) == 1:
                stop_time = time.time()
                if stop_time - start_time > timeout:
                    return 0.0
                    
            time_elapsed = stop_time - start_time
            distance = (time_elapsed * 34300) / 2
            
            return distance


def main(args=None):
    rclpy.init(args=args)
    distance_sensor_publisher = DistanceSensorPublisher()
    try:
        rclpy.spin(distance_sensor_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        distance_sensor_publisher.destroy_node()
        rclpy.shutdown()
        GPIO.cleanup()


if __name__ == '__main__':
    main()
