import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int8, UInt8
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from math import sqrt, isinf, isnan, fmod, pi

class PoseDistanceNode(Node):
    def __init__(self):
        super().__init__('pose_distance_node')
        self.bve_pose = None
        self.enemy_pose = None
        self.scan_ranges = None
        self.scan_angle_min = None
        self.scan_angle_increment = None
        self.zero_published = False  # Flag to track if zero velocity was published
        self.is_activated = False  # Flag to track if node is activated via /timer

        self.pose_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Define angular exclusion ranges (in radians)
        self.exclude_range_1_start = 0  # 0 to π/4
        self.exclude_range_1_end = pi * 4 / 16
        self.exclude_range_2_start = pi * 28 / 16  # 7π/4 to 2π
        self.exclude_range_2_end = 2 * pi

        # Subscribers
        self.bve_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/bve_pose',
            self.bve_pose_callback,
            qos_profile=self.pose_qos)
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
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.timer_sub = self.create_subscription(
            UInt8,
            '/timer',
            self.timer_callback,
            10)

        # Publishers
        self.cmd_vel_filtered_pub = self.create_publisher(Twist, '/cmd_vel/filtered', 10)
        self.enemy_warning_pub = self.create_publisher(Int8, '/enemy_warning', 10)

        # Timer for periodic distance check
        self.timer = self.create_timer(0.01, self.check_distance)

    def bve_pose_callback(self, msg):
        self.bve_pose = msg.pose

    def enemy_pose_callback(self, msg):
        self.enemy_pose = msg.pose

    def scan_callback(self, msg):
        # self.get_logger().info("lidar callback")
        self.scan_ranges = msg.ranges
        self.scan_angle_min = msg.angle_min
        self.scan_angle_increment = msg.angle_increment

    def timer_callback(self, msg):
        if msg.data == 1 and not self.is_activated:
            self.is_activated = True
            self.get_logger().info("Node activated via /timer")

    def get_scan_distance(self):
        if self.scan_ranges is None or self.scan_angle_min is None or self.scan_angle_increment is None:
            # self.get_logger().info("Some value is None")
            return float('inf')

        # self.get_logger().info("lidar callback")
        valid_ranges = []
        for i, r in enumerate(self.scan_ranges):
            if isinf(r) or isnan(r) or r < 0.27:  # Filter out invalid ranges and < 25 cm
                continue

            # Calculate the angle for this range
            angle = self.scan_angle_min + i * self.scan_angle_increment

            # Normalize angle to [0, 2π)
            angle = fmod(angle + 2 * pi, 2 * pi)

            # Check if angle is in exclusion ranges
            in_exclusion = False
            if self.exclude_range_1_start != -1000.0 and self.exclude_range_1_end != -1000.0:
                range_1_start = fmod(self.exclude_range_1_start + 2 * pi, 2 * pi)
                range_1_end = fmod(self.exclude_range_1_end + 2 * pi, 2 * pi)
                if range_1_start <= range_1_end:
                    if range_1_start <= angle <= range_1_end:
                        in_exclusion = True
                else:
                    if angle >= range_1_start or angle <= range_1_end:
                        in_exclusion = True

            if self.exclude_range_2_start != -1000.0 and self.exclude_range_2_end != -1000.0:
                range_2_start = fmod(self.exclude_range_2_start + 2 * pi, 2 * pi)
                range_2_end = fmod(self.exclude_range_2_end + 2 * pi, 2 * pi)
                if range_2_start <= range_2_end:
                    if range_2_start <= angle <= range_2_end:
                        in_exclusion = True
                else:
                    if angle >= range_2_start or angle <= range_2_end:
                        in_exclusion = True

            if not in_exclusion:
                valid_ranges.append(r)

        return min(valid_ranges) if valid_ranges else float('inf')

    def cmd_vel_callback(self, msg):
        if not self.is_activated:
            return

        # if self.bve_pose is None or self.enemy_pose is None:
        #     self.cmd_vel_filtered_pub.publish(msg)
        #     return

        # # Calculate pose-based distance
        # dx = self.bve_pose.pose.position.x - self.enemy_pose.pose.position.x
        # dy = self.bve_pose.pose.position.y - self.enemy_pose.pose.position.y
        # pose_distance = sqrt(dx**2 + dy**2)

        # self.get_logger().info(f"Scan distance = ")

        # Get scan-based distance
        scan_distance = self.get_scan_distance()

        # If scan distance is >= 35 cm, forward cmd_vel to cmd_vel/filtered
        if scan_distance >= 0.3:
            self.cmd_vel_filtered_pub.publish(msg)
            self.zero_published = False  # Reset flag when enemy is far

    def check_distance(self):
        if not self.is_activated:
            return

        # if self.bve_pose is None or self.enemy_pose is None:
        #     return

        # # Calculate pose-based distance
        # dx = self.bve_pose.pose.position.x - self.enemy_pose.pose.position.x
        # dy = self.bve_pose.pose.position.y - self.enemy_pose.pose.position.y
        # pose_distance = sqrt(dx**2 + dy**2)

        # Get scan-based distance
        scan_distance = self.get_scan_distance()
        self.get_logger().info(f"Scan distance = {scan_distance}")

        # Publish enemy warning: 1 if scan distance is >= 30 cm, 0 if < 30 cm
        warning_msg = Int8()
        warning_msg.data = 1 if scan_distance >= 0.3 else 0
        # self.enemy_warning_pub.publish(warning_msg)

        # If scan distance is < 35 cm, publish zero velocity once
        if scan_distance < 0.3 and not self.zero_published:
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