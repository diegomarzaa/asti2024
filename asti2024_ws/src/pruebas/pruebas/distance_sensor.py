#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO_TRIGGER = 23
GPIO_ECHO = 24

GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

class DistanceSensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_distancia_node')
        self.publisher_ = self.create_publisher(Float32, 'sensor_distancia', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_sensor_data)
        GPIO.output(GPIO_TRIGGER, False)

    def publish_sensor_data(self):
        msg = Float32()
        
        # ENCONTRAMOS LA MEDIDA
        # Emitimos el pulso
        GPIO.output(GPIO_TRIGGER, True)
        time.sleep(0.00001)
        GPIO.output(GPIO_TRIGGER, False)
        
        StartTime = time.time()
        StopTime = time.time()
        
        # save StartTime
        while GPIO.input(GPIO_ECHO) == 0:
            StartTime = time.time()
    
        # save time of arrival
        while GPIO.input(GPIO_ECHO) == 1:
            StopTime = time.time()
            
        # time difference between start and arrival
        TimeElapsed = StopTime - StartTime
        # multiply with the sonic speed (34300 cm/s)
        # and divide by 2, because there and back
        distance = (TimeElapsed * 34300) / 2
        
        msg.data = distance

        # Publica el mensaje

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

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
