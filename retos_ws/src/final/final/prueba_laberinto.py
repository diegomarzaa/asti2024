from final.Sensors import Sensors
import rclpy
from rclpy.node import Node
from final.Movements import Movements
from time import sleep

class MazeSolver(Node):
    def __init__(self):
        super().__init__('maze_solver')
        self.sensors = Sensors()
        self.mov = Movements()
        self.always_right_wall_following_strategy()

    def always_right_wall_following_strategy(self):
        while rclpy.ok():
            # If there is a wall on the right side and in front, move left
            if self.sensors.distancia_der < 5 and self.sensors.distancia_delante_der < 10: 
                self.mov.girar_izquierda()
            # If there's no wall on the right side, move right
            elif self.sensors.distancia_der > 5:
                self.mov.girar_derecha()
            # Else, keep moving forward
            else:
                self.mov.avanzar()
            sleep(0.1)  
            rclpy.spin(self.sensors)
            
def main(args=None):
    rclpy.init(args=args)
    maze_solver = MazeSolver()
    rclpy.spin(maze_solver)
    maze_solver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

