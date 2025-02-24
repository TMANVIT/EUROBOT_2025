import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
import numpy as np
from scipy.spatial.transform import Rotation as R



class MapToOdomTF(Node):
    def __init__(self):
        super().__init__("camera_odom")

        self.create_subscription(
            PoseStamped, "/bev_pose", self.odom_calculation, 10
        )
        self.create_subscription(
            PoseStamped, "/initialpose", self.initial_pose_initialisation, 10
        )
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.inital_pose_c = None
        self.inital_pose_q = None
        self.aruco_to_base_link = None
        self.map_to_aruco = None

    def odom_calculation(self, msg):
        if self.inital_pose is not None:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "odom"
            t.child_frame_id = "base_footprint"

            map_c = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
            map_q = R.from_quat(np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]))



            odom_to_base_footprint = self.tf_buffer.transform(t, "odom")
            
            self.tf_broadcaster.sendTransform(odom_to_base_footprint)


            self.get_logger().info(
                f"odom pose set: x={initial_x}, y={initial_y}, z={initial_z}"
            )

    def initial_pose_initialisation(self, msg):
        self.inital_pose_c = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self.inital_pose_q = R.from_quat(np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]))





def main(args=None):
    rclpy.init(args=args)
    node = MapToOdomTF()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()