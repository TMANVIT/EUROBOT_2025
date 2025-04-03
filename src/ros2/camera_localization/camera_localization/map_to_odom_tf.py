import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
import numpy as np
from scipy.spatial.transform import Rotation as R



class MapToOdomTF(Node):
    def __init__(self):
        super().__init__("map_to_odom_tf")
        
        self.pose_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self.initial_pose_callback,
            qos_profile=self.pose_qos
        )
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread = True)
        self.timer = self.create_timer(0.1, self.broadcast_map_to_odom)
        self.map_to_odom = None

    def initial_pose_callback(self, msg):

        self.map_to_odom = TransformStamped()
        self.map_to_odom.header.stamp = self.get_clock().now().to_msg()
        self.map_to_odom.header.frame_id = "map"
        self.map_to_odom.child_frame_id = "odom"

        self.map_to_odom.transform.translation.x = msg.pose.pose.position.x
        self.map_to_odom.transform.translation.y = msg.pose.pose.position.y
        self.map_to_odom.transform.translation.z = msg.pose.pose.position.z

        self.map_to_odom.transform.rotation.x = msg.pose.pose.orientation.x
        self.map_to_odom.transform.rotation.y = msg.pose.pose.orientation.y
        self.map_to_odom.transform.rotation.z = msg.pose.pose.orientation.z
        self.map_to_odom.transform.rotation.w = msg.pose.pose.orientation.w
    
        self.tf_broadcaster.sendTransform(self.map_to_odom)
   
    def broadcast_map_to_odom(self):
        if self.map_to_odom is not None:
            self.map_to_odom.header.stamp = self.get_clock().now().to_msg()
            self.tf_broadcaster.sendTransform(self.map_to_odom)

def main(args=None):
    rclpy.init(args=args)
    node = MapToOdomTF()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()