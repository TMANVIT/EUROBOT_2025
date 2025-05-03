import rclpy
from rclpy.node import Node
from obstacle_detector.msg import Obstacles
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist

NUM_LANDMARKS = 3


class LidarLocalization(Node):
    def __init__(self):
        super().__init__("lidar_localization_node")
        
        self.pose_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.create_subscription(
            Obstacles, "/raw_obstacles", self.obstacle_callback, 10
        )
        
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        
        self.cmd_vel_filtered_pub = self.create_publisher(Twist, '/cmd_vel/filtered', 10)
        self.stop = True

    def obstacle_callback(self, msg):
        # obstacles = [np.array([obs.center.x, obs.center.y]) for obs in msg.circles]
        
        # for obst in obstacles:
        
        for obs in msg.circles:
            if obs.center.x < 0.4 and obs.center.x > 0.1 and obs.center.y < 0.4 and obs.center.y > 0.1:
                self.stop = False
                
    def cmd_vel_callback(self, msg):

        # If enemy is far (>= 20 cm), forward cmd_vel to cmd_vel/filtered
        if self.stop:
            self.cmd_vel_filtered_pub.publish(msg)
        else:
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_filtered_pub.publish(twist)
                
                

def main(args=None):
    rclpy.init(args=args)
    node = LidarLocalization()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
