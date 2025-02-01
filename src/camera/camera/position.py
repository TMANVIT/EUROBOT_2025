import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose

class BEVPosePublisher(Node):

    def __init__(self):
        super().__init__('bev_pose_publisher')
        self.publisher_ = self.create_publisher(Pose, 'bev_pose', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Pose()
        msg.position.x = 1.0
        msg.position.y = 2.0
        msg.position.z = 3.0
        msg.orientation.x = 11.0
        msg.orientation.y = 12.0
        msg.orientation.z = 13.0
        msg.orientation.w = 14.0
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    bve_pose_publisher = BEVPosePublisher()

    rclpy.spin(bve_pose_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    bve_pose_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()