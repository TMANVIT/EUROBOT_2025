import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
import numpy as np
from scipy.spatial.transform import Rotation as R



class MapToOdomTF(Node):
    def __init__(self):
        super().__init__("map_to_odom_tf")

        self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self.initial_pose_callback, 10
        )
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread = True)
        self.timer = self.create_timer(0.1, self.broadcast_map_to_odom)
        self.aruco_to_base_link = None
        self.map_to_aruco = None
        self.init_pose = False

    def initial_pose_callback(self, msg):
        if not self.init_pose:
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

            self.init_pose = True
            
        self.tf_broadcaster.sendTransform(self.map_to_odom)

    
    def broadcast_map_to_odom(self):
        if self.init_pose:
            self.map_to_odom.header.stamp = self.get_clock().now().to_msg()
            self.tf_broadcaster.sendTransform(self.map_to_odom)
            self.get_logger().info(f"Initial pose set: x={self.map_to_odom}")




def main(args=None):
    rclpy.init(args=args)
    node = MapToOdomTF()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()