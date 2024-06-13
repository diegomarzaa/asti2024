import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool 
from gpiozero import DistanceSensor

class DistanceSensorPublisher(Node):
    def __init__(self):
        super().__init__('distance_sensor_publisher')
        self.publisher_ = self.create_publisher(Bool, 'obstacle', 10)  # Changed to Bool
        
        # Declare parameters and get their initial values
        self.declare_parameter('echo_pin', 23)
        self.declare_parameter('trigger_pin', 24)
        self.declare_parameter('distancia_minima', 0.0)  
        self.declare_parameter('distancia_limite', 0.2)  
        echo_pin = self.get_parameter('echo_pin').get_parameter_value().integer_value
        trigger_pin = self.get_parameter('trigger_pin').get_parameter_value().integer_value
        self.distancia_minima = self.get_parameter('distancia_minima').get_parameter_value().double_value  # Added minimum distance parameter
        self.distancia_limite = self.get_parameter('distancia_limite').get_parameter_value().double_value  # Corrected parameter name

        # Initialize the DistanceSensor with parameters
        self.sensor = DistanceSensor(echo=echo_pin, trigger=trigger_pin)

        timer_period = 0.001  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        distance_measured = self.sensor.distance
        distance_within_limit = distance_measured <= self.distancia_limite and distance_measured >= self.distancia_minima
        self.publisher_.publish(Bool(data=distance_within_limit))  # Publish True or False
        # Updated log message to include distance measured
        self.get_logger().info(f'Distance: {distance_measured:.2f} m, within limit: {distance_within_limit}')

def main(args=None):
    rclpy.init(args=args)
    distance_sensor_publisher = DistanceSensorPublisher()
    rclpy.spin(distance_sensor_publisher)
    distance_sensor_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
