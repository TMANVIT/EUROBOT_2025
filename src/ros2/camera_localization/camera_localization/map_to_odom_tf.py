import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
import numpy as np
from scipy.spatial.transform import Rotation as R



class MapToOdomTF(Node):
    def __init__(self):
        super().__init__("map_to_odom_tf")

        self.create_subscription(
            PoseStamped, "/initialpose", self.initial_pose_callback, 10
        )
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread = True)
        self.aruco_to_base_link = None
        self.map_to_aruco = None

    def initial_pose_callback(self, msg):

        self.map_to_aruco = TransformStamped()
        self.map_to_aruco.header.stamp = self.get_clock().now().to_msg()
        self.map_to_aruco.header.frame_id = "map"
        self.map_to_aruco.child_frame_id = "odom"

        self.map_to_aruco.transform.translation.x = msg.pose.position.x
        self.map_to_aruco.transform.translation.y = msg.pose.position.y
        self.map_to_aruco.transform.translation.z = msg.pose.position.z

        self.map_to_aruco.transform.rotation.x = msg.pose.orientation.x
        self.map_to_aruco.transform.rotation.y = msg.pose.orientation.y
        self.map_to_aruco.transform.rotation.z = msg.pose.orientation.z
        self.map_to_aruco.transform.rotation.w = msg.pose.orientation.w

        initial_x = msg.pose.position.x
        initial_y = msg.pose.position.y
        initial_z = msg.pose.position.z

    
        self.aruco_to_base_link = self.tf_buffer.lookup_transform("aruco_link", "base_link", rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=2.0))

        map_to_odom = self.compose_transforms(self.map_to_aruco, self.aruco_to_base_link)
        self.tf_broadcaster.sendTransform(map_to_odom)


        self.get_logger().info(
            f"Initial pose set: x={initial_x}, y={initial_y}, z={initial_z}"
        )

    

    def compose_transforms(self, first, second):
        composed = TransformStamped()
        composed.header.stamp = self.get_clock().now().to_msg()
        composed.header.frame_id = "map"
        composed.child_frame_id = "odom"

        # Получаем данные
        first_trans = np.array([first.transform.translation.x,
                       first.transform.translation.y,
                       first.transform.translation.z])
        first_rot = R.from_quat([first.transform.rotation.x,
                     first.transform.rotation.y,
                     first.transform.rotation.z,
                     first.transform.rotation.w])

        second_trans = np.array([second.transform.translation.x,
                        second.transform.translation.y,
                        second.transform.translation.z])
        second_rot = R.from_quat([second.transform.rotation.x,
                      second.transform.rotation.y,
                      second.transform.rotation.z,
                      second.transform.rotation.w])

        # Применяем вращение и трансляцию
        new_trans = first_trans+first_rot.apply(second_trans)
        new_rot = first_rot * second_rot

        # Записываем в сообщение
        composed.transform.translation.x = new_trans[0]
        composed.transform.translation.y = new_trans[1]
        composed.transform.translation.z = new_trans[2]

        # new_quat = new_rot.as_quat()
        new_quat = first_rot.as_quat()
        composed.transform.rotation.x = new_quat[0]
        composed.transform.rotation.y = new_quat[1]
        composed.transform.rotation.z = new_quat[2]
        composed.transform.rotation.w = new_quat[3]

        return composed




def main(args=None):
    rclpy.init(args=args)
    node = MapToOdomTF()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()