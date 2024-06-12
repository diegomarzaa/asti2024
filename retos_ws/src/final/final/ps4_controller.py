import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from final.Movements import Movements, pedir_velocidades

obstacle_front_left = False
obstacle_front_right = False
obstacle_left = False
obstacle_right = False
class PS4Node(Node):
    def __init__(self):
        super().__init__('ps4_controller')

        self.mov = Movements()
        self.mov.actualizar_vel_lineal(0.5)
        self.mov.actualizar_vel_angular(1.2)

        self.multiplicador_linear_vel = 1.0
        self.multiplicador_angular_vel = 1.0
        
        self.aceleracion = 0.0
        self.deceleracion = 0.0

        self.subs_joy = self.create_subscription(Joy, 'joy', self.callback_joy, 10)
        
        self.timer = self.create_timer(0.001, self.loop)  # 0.001 seconds

    def callback_joy(self, msg):

        # ------ BOTONES -------
        if msg.buttons[0] == 1:
            print("CUADRADO")
        elif msg.buttons[1] == 1:
            print("X")
        elif msg.buttons[2] == 1:
            print("CIRCULO")
        elif msg.buttons[3] == 1:
            print("TRIANGULO")
        
        # ------ JOYSTICKS -------

        # Joystick izquierdo
        if msg.axes[0] < 0:
            print("DERECHA (joystick izquierdo): ", msg.axes[0])
        elif msg.axes[0] > 0:
            print("IZQUIERDA (joystick izquierdo): ", msg.axes[0])
        if msg.axes[1] < 0:
            print("ABAJO (joystick izquierdo): ", msg.axes[1])
        elif msg.axes[1] > 0:
            print("ARRIBA (joystick izquierdo): ", msg.axes[1])

        
        # Joystick derecho
        if msg.axes[2] < 0:
            print("DERECHA (joystick derecho): ", msg.axes[2])
        elif msg.axes[2] > 0:
            print("IZQUIERDA (joystick derecho): ", msg.axes[2])
        if msg.axes[5] < 0:
            print("ABAJO (joystick derecho): ", msg.axes[5])
        elif msg.axes[5] > 0:
            print("ARRIBA (joystick derecho): ", msg.axes[5])

        # Acelerador R2
        if msg.axes[4] < 0.7:
            self.aceleracion = (msg.axes[4]-0.7)*(-2.5) / 10    # 0.7 -> 0.0, -1.0 -> 0.425  // Es una aceleración trivial desde 0 a 4.25
        elif msg.axes[4] > 0.7:
            self.aceleracion = 0.0

        # Decelerador L2
        if msg.axes[3] < 0.7:
            self.deceleracion = (msg.axes[3]-0.7)*(-2) / 10    # 0.7 -> 0.0, -1.0 -> 0.34  // Es una aceleración trivial desde 0 a 4.25
        elif msg.axes[3] > 0.7:
            self.deceleracion = 0.0

        # Set velocidades
        self.multiplicador_linear_vel = msg.axes[1]
        self.multiplicador_angular_vel = msg.axes[0]

    def loop(self):
        if self.aceleracion > 0:
          vel_lineal = self.multiplicador_linear_vel * self.mov.obj_linear_vel + self.aceleracion
        else:
          vel_lineal = self.multiplicador_linear_vel * self.mov.obj_linear_vel - self.deceleracion
        
        vel_angular = self.multiplicador_angular_vel * self.mov.obj_angular_vel
        
        if vel_lineal < 0:
            vel_angular = -vel_angular    # al ir marcha atrás girará al revés

        self.mov.avanzar_curva(vel_lineal, vel_angular)

def main(args=None):
    rclpy.init(args=args)
    avoidance_node = PS4Node()
    mov = Movements()
    mov.actualizar_vel_lineal(0.4)
    mov.actualizar_vel_angular(0.3)
    rclpy.spin(avoidance_node)
    avoidance_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
