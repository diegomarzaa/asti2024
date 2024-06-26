#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
#import RPi.GPIO as GPIO
import time

#GPIO.setmode(GPIO.BCM)
#GPIO_TRIGGERder = 23
#GPIO_ECHOder = 24
#GPIO_TRIGGERizq = 20
#GPIO_ECHOizq = 21



#GPIO.setup(GPIO_TRIGGERder, GPIO.OUT)
#GPIO.setup(GPIO_ECHOder, GPIO.IN)
#GPIO.setup(GPIO_TRIGGERizq, GPIO.OUT)
#GPIO.setup(GPIO_ECHOizq, GPIO.IN)

class DistanceSensorPublisher(Node):
    
        def __init__(self):
            super().__init__('distance_sensor_publisher')
            self.publisher_der = self.create_publisher(Float32, 'distance_der', 10)
            self.publisher_izq = self.create_publisher(Float32, 'distance_izq', 10)
            self.publisher_delante = self.create_publisher(Float32, 'distance_delante', 10)
            timer_period = 0.8  # seconds
            self.timer = self.create_timer(timer_period, self.timer_callback)
            self.distance_der = 35.0
            self.distance_izq = 35.0
            self.distance_delante = 30.0
    
        def timer_callback(self):
            #distance_der = self.distance(GPIO_TRIGGERder, GPIO_ECHOder)
            #distance_izq = self.distance(GPIO_TRIGGERizq, GPIO_ECHOizq)
            
            self.distance_der -= 1.0
            self.distance_izq -= 1.0
            self.distance_delante -= 1.0

            msg_der = Float32()
            if self.distance_der is not None and self.distance_der > 0.0:
                msg_der.data = self.distance_der
            else:
                msg_der.data = 0.0
            self.publisher_der.publish(msg_der)
            print("Sensor derecha: \t", msg_der.data)
            #self.get_logger().info('Sensor derecha: \t"%s"' % msg_der.data)
            
            msg_izq = Float32()
            if self.distance_izq is not None and self.distance_izq > 0.0:
                msg_izq.data = self.distance_izq
            else:
                msg_izq.data = 0.0
            self.publisher_izq.publish(msg_izq)
            print("Sensor izquierda: \t", msg_izq.data)
            #self.get_logger().info('Sensor izquierda: \t"%s" \n' % msg_izq.data)
            
            msg_delante = Float32()
            if self.distance_delante is not None and self.distance_delante > 0.0:
                msg_delante.data = self.distance_delante
            else:
                msg_delante.data = 0.0
            self.publisher_delante.publish(msg_delante)
            print("Sensor delante: \t", msg_delante.data)
            print("\n")
            #self.get_logger().info('Sensor izquierda: \t"%s" \n' % msg_izq.data)
    
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
