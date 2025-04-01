import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

class MapToOdomTF(Node):
    def __init__(self):
        super().__init__("map_to_odom_tf")

        # Set up QoS profile for TF compatibility
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        # Create subscription with compatible QoS
        self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            qos_profile=qos_profile
        )

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, self, spin_thread=True
        )
        
        # Timer for periodic transform publishing
        self.timer = self.create_timer(0.02, self.broadcast_map_to_odom)
        self.init = False

        # Initialize transform message
        self.map_to_odom = TransformStamped()
        self.map_to_odom.header.frame_id = "map"
        self.map_to_odom.child_frame_id = "odom"

    def odom_callback(self, msg):
        # Update transform from odometry message
        self.map_to_odom.header.stamp = self.get_clock().now().to_msg()
        
        self.map_to_odom.transform.translation.x = msg.pose.pose.position.x
        self.map_to_odom.transform.translation.y = msg.pose.pose.position.y
        self.map_to_odom.transform.translation.z = msg.pose.pose.position.z

        self.map_to_odom.transform.rotation.x = msg.pose.pose.orientation.x
        self.map_to_odom.transform.rotation.y = msg.pose.pose.orientation.y
        self.map_to_odom.transform.rotation.z = msg.pose.pose.orientation.z
        self.map_to_odom.transform.rotation.w = msg.pose.pose.orientation.w
        
        self.init = True
        self.tf_broadcaster.sendTransform(self.map_to_odom)

    def broadcast_map_to_odom(self):
        if self.init:
            # Update timestamp and broadcast
            self.map_to_odom.header.stamp = self.get_clock().now().to_msg()
            self.tf_broadcaster.sendTransform(self.map_to_odom)

def main(args=None):
    rclpy.init(args=args)
    node = MapToOdomTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()