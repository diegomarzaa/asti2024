import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool 
import random

obstacle_front_left = False
obstacle_front_right = False
obstacle_left = False
obstacle_right = False
class AvoidanceNode(Node):
    def __init__(self):
        super().__init__('avoidance')
        self.state = 1  # 1: forward, 2: turn left, 3: turn right
        self.obstacle_front_left = False
        self.obstacle_front_right = False
        self.obstacle_left = False
        self.obstacle_right = False
        
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subs_front_left = self.create_subscription(Bool, 'front_left/obstacle', self.callback_front_left, 10)
        self.subs_front_right = self.create_subscription(Bool, 'front_right/obstacle', self.callback_front_right, 10)
        self.subs_left = self.create_subscription(Bool, 'left/obstacle', self.callback_left, 10)
        self.subs_right = self.create_subscription(Bool, 'right/obstacle', self.callback_right, 10)
        
        self.timer = self.create_timer(0.001, self.loop)  # 50ms loop rate

    def callback_front_left(self, msg):
        #print("detector front left: ", msg.data)
        self.obstacle_front_left = msg.data

    def callback_front_right(self, msg):
        #print("detector front right: ", msg.data)
        self.obstacle_front_right = msg.data

    def callback_left(self, msg):
        #print("detector left: ", msg.data)
        self.obstacle_left = msg.data

    def callback_right(self, msg):
        #print("detector right: ", msg.data)
        self.obstacle_right = msg.data

    def loop(self):
        message = Twist()
        print("obstacle front left: ", self.obstacle_front_left)
        print("obstacle front right: ", self.obstacle_front_right)
        print("obstacle left: ", self.obstacle_left)
        print("obstacle right: ", self.obstacle_right)
        
        if self.obstacle_front_left:
            message.linear.x = 0.0
            message.angular.z = -0.5 # turn right
        elif self.obstacle_front_right:
            message.linear.x = 0.0
            message.angular.z = 0.5 # turn left
        elif self.obstacle_right:
            message.linear.x = 0.15
            message.angular.z = 0.25  # turn left (poquillo)
        elif self.obstacle_left:
            message.linear.x = 0.15
            message.angular.z = -0.25  # turn right (poquillo)
        else:
            message.linear.x = 0.2
            message.angular.z = 0.0

        self.publisher.publish(message)

def main(args=None):
    rclpy.init(args=args)
    avoidance_node = AvoidanceNode()
    rclpy.spin(avoidance_node)
    avoidance_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
