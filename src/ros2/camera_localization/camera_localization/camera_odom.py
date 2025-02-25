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
        self.inital_pose_coords = None
        self.inital_pose_rotation = None

    def odom_calculation(self, msg):
        if self.inital_pose_c is not None:
            odom_to_base_footprint = TransformStamped()
            odom_to_base_footprint.header.stamp = self.get_clock().now().to_msg()
            odom_to_base_footprint.header.frame_id = "odom"
            odom_to_base_footprint.child_frame_id = "base_footprint"

            map_coodrinats = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
            map_rotation = (R.from_quat(np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]))).as_matrix()

            odom_to_base_footprint_coodrs = np.invert(self.inital_pose_rotation) @ (map_coodrinats-self.map_coodrinats)
            odom_to_base_footprint_quat = (R.from_matrix(map_rotation @ np.invert(self.inital_pose_rotation))).as_matrix()

            odom_to_base_footprint.transform.translation.x = odom_to_base_footprint_coodrs[0]
            odom_to_base_footprint.transform.translation.y = odom_to_base_footprint_coodrs[1]
            odom_to_base_footprint.transform.translation.z = odom_to_base_footprint_coodrs[2]

            odom_to_base_footprint.transform.rotation.x = odom_to_base_footprint_quat[0]
            odom_to_base_footprint.transform.rotation.y = odom_to_base_footprint_quat[1]
            odom_to_base_footprint.transform.rotation.z = odom_to_base_footprint_quat[2]
            odom_to_base_footprint.transform.rotation.w = odom_to_base_footprint_quat[3]
            
            self.tf_broadcaster.sendTransform(odom_to_base_footprint)



            # self.get_logger().info(
            #     f"odom pose set: x={initial_x}, y={initial_y}, z={initial_z}"
            # )

    def initial_pose_initialisation(self, msg):
        self.inital_pose_coords = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self.inital_pose_rotation = (R.from_quat(np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]))).as_matrix()





def main(args=None):
    rclpy.init(args=args)
    node = MapToOdomTF()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()