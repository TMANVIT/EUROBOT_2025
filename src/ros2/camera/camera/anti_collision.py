import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from std_msgs.msg import Int8
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from math import sqrt

class PoseDistanceNode(Node):
    def __init__(self):
        super().__init__('pose_distance_node')
        self.bve_pose = None
        self.enemy_pose = None
        self.zero_published = False  # Flag to track if zero velocity was published
        
        self.pose_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.timer_sub = self.create_subscription(UInt8, '/timer', self.timer_counter, 10)
        
        # Subscribers
        self.bve_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/bve_pose',
            self.bve_pose_callback,
            qos_profile=self.pose_qos
            )
        self.enemy_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/enemy_pose',
            self.enemy_pose_callback,
            qos_profile=self.pose_qos)
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        
        # Publishers
        self.cmd_vel_filtered_pub = self.create_publisher(Twist, '/cmd_vel/filtered', 10)
        self.enemy_warning_pub = self.create_publisher(Int8, '/enemy_warning', 10)
        
        # Timer for periodic distance check
        self.timer = self.create_timer(0.01, self.check_distance)
        self.start_timer = None 

    def bve_pose_callback(self, msg):
        self.bve_pose = msg.pose

    def enemy_pose_callback(self, msg):
        self.enemy_pose = msg.pose

    def timer_counter(self, msg):
        if self.start_timer is None and msg.data == 1:
            self.start_timer = time.time()

    def cmd_vel_callback(self, msg):
        if self.enemy_pose is None:
            self.cmd_vel_filtered_pub.publish(msg)
            return

        # Calculate distance between poses
        dx = self.bve_pose.pose.position.x - self.enemy_pose.pose.position.x
        dy = self.bve_pose.pose.position.y - self.enemy_pose.pose.position.y
        distance = sqrt(dx**2 + dy**2)

        # If enemy is far (>= 20 cm), forward cmd_vel to cmd_vel/filtered
        if distance >= 0.4 and (time.time() - self.start_timer) < 101:
            self.cmd_vel_filtered_pub.publish(msg)
            self.zero_published = False  # Reset flag when enemy is far

    def check_distance(self):
        if self.bve_pose is None or self.enemy_pose is None:
            return

        # Calculate distance between poses
        dx = self.bve_pose.pose.position.x - self.enemy_pose.pose.position.x
        dy = self.bve_pose.pose.position.y - self.enemy_pose.pose.position.y
        distance = sqrt(dx**2 + dy**2)

        # Publish enemy warning: 1 if enemy is outside 20 cm, 0 if inside
        warning_msg = Int8()
        warning_msg.data = 1 if distance >= 0.4 else 0
        self.enemy_warning_pub.publish(warning_msg)

        # If enemy is close (< 20 cm), publish zero velocity once
        if distance < 0.4 and not self.zero_published and (time.time() - self.start_timer) > 100:
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_filtered_pub.publish(twist)
            self.zero_published = True

def main(args=None):
    rclpy.init(args=args)
    node = PoseDistanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()