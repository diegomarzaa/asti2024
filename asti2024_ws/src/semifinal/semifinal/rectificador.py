import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class Rectificador(Node):
    def __init__(self):
        super().__init__('rectificador')
        self.publisher_ = self.create_publisher(Int32, 'rectificar', 10)
        self.publisher_

        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.rectificar()

    def rectificar(self):

        msg = Int32()
        msg.data = int(input("Introduce un número: \n"
                             "0 para avanzar\n"
                             "-1 para girar a la izquierda\n"
                             "1 para girar a la derecha\n"))
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')


def main(args=None):
    rclpy.init(args=args)

    rectificador = Rectificador()

    rclpy.spin(rectificador)

    rectificador.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()