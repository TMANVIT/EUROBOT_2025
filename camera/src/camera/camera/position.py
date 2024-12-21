import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose

class CameraPublisher(Node):

    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Pose, 'camera_topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Pose()
        msg.position.x = 1
        msg.position.y = 2
        msg.position.z = 3
        msg.orientation.x = 11
        msg.orientation.y = 12
        msg.orientation.z = 13
        msg.orientation.w = 14
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = CameraPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()