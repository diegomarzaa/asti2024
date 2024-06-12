import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool 

obstacle_front_left = False
obstacle_front_right = False
obstacle_left = False
obstacle_right = False
class PS4Node(Node):
    def __init__(self):
        super().__init__('ps4_controller')
        self.state = "quieto"
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subs_joy = self.create_subscription(Joy, 'joy', self.callback_joy, 10)
        
        self.timer = self.create_timer(0.001, self.loop)  # 0.001 seconds

    def callback_joy(self, msg):

        # ------ BOTONES -------
        if msg.buttons[0] == 1:
            print("CUADRADO")
        elif msg.buttons[1] == 1:
            print("X")
            self.state = "quieto"
        elif msg.buttons[2] == 1:
            print("CIRCULO")
        elif msg.buttons[3] == 1:
            print("TRIANGULO")
        
        # ------ JOYSTICKS -------

        # Joystick izquierdo
        if msg.axes[0] < 0:
            print("DERECHA (joystick izquierdo): ", msg.axes[0])
            self.state = "girar_derecha"
        elif msg.axes[0] > 0:
            print("IZQUIERDA (joystick izquierdo): ", msg.axes[0])
            self.state = "girar_izquierda"
        if msg.axes[1] < 0:
            print("ABAJO (joystick izquierdo): ", msg.axes[1])
            self.state = "retroceder"
        elif msg.axes[1] > 0:
            print("ARRIBA (joystick izquierdo): ", msg.axes[1])
            self.state = "avanzar"

        
        # Joystick derecho
        if msg.axes[2] < 0:
            print("DERECHA (joystick derecho): ", msg.axes[2])
        elif msg.axes[2] > 0:
            print("IZQUIERDA (joystick derecho): ", msg.axes[2])
        if msg.axes[5] < 0:
            print("ABAJO (joystick derecho): ", msg.axes[5])
        elif msg.axes[5] > 0:
            print("ARRIBA (joystick derecho): ", msg.axes[5])

    def loop(self):
        message = Twist()

        if self.state == "avanzar":
            message.linear.x = 0.2
            message.angular.z = 0.0
        elif self.state == "retroceder":
            message.linear.x = -0.2
            message.angular.z = 0.0
        elif self.state == "girar_derecha":
            message.linear.x = 0.0
            message.angular.z = -0.2
        elif self.state == "girar_izquierda":
            message.linear.x = 0.0
            message.angular.z = 0.2
        elif self.state == "quieto":
            message.linear.x = 0.0
            message.angular.z = 0.0

        self.publisher.publish(message)

def main(args=None):
    rclpy.init(args=args)
    avoidance_node = PS4Node()
    rclpy.spin(avoidance_node)
    avoidance_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
