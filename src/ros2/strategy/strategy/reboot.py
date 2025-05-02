import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class reInit(Node):
    def __init__(self):
        super().__init__('reboot_publisher')
        self.odom_subscriber = self.create_subscription(Odometry,'/odom/unfiltered', self.odom_callback, 10)
        self.last_odom_publisher = self.create_publisher(Odometry, '/odom/reboot', 10)
        self.last_odometry = None
        self.timer = self.create_timer(0.5, self.timer_callback)
        
    def odom_callback(self, msg):
        self.last_odometry = msg
        
    def timer_callback(self):
        if hasattr(self, 'last_odometry') and self.last_odometry is not None:
            self.last_odom_publisher.publish(self.last_odometry)

def main(args=None):
    rclpy.init(args=args)
    node = reInit()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()