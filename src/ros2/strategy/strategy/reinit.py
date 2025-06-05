import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class reInit(Node):
    def __init__(self):
        super().__init__('reinit_publisher')
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.declare_parameter('oneshot', False)
        
    def timer_callback(self):
        cmd_vel_msg = Twist()
        
        cmd_vel_msg.linear.x = 0.0
        cmd_vel_msg.linear.y = 0.0
        cmd_vel_msg.linear.z = 0.0

        cmd_vel_msg.angular.x = 0.0
        cmd_vel_msg.angular.y = 0.0
        cmd_vel_msg.angular.z = 0.0
        
        self.cmd_vel_publisher.publish(cmd_vel_msg)
        
        if self.get_parameter('oneshot').value:
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = reInit()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()