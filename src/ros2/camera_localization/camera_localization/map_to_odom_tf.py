import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf_transformations as tf

class MapToOdomTF(Node):
    def __init__(self):
        super().__init__("map_to_odom_tf")

        self.create_subscription(
            PoseStamped, "/initialpose", self.initial_pose_callback, 10
        )
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.aruco_to_odom  = self.tf_buffer.lookup_transform("aruco", "base_link", rclpy.time.Time())
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

        map_to_odom = self.compose_transforms(self.map_to_aruco, self.aruco_to_odom)
        self.tf_broadcaster.sendTransform(map_to_odom)


        self.get_logger().info(
            f"Initial pose set: x={initial_x}, y={initial_y}, z={initial_z}"
        )

        


    def get_aruco_to_odom_tf(self):
        aruco_to_odom =  self.tf_buffer.lookup_transform("aruco", "odom", rclpy.time.Time())
        map_to_odom = self.compose_transforms(self.map_to_aruco, aruco_to_odom)
        self.tf_broadcaster.sendTransform(map_to_odom)

    def compose_transforms(self, first, second):
        composed = TransformStamped()
        composed.header.stamp = self.get_clock().now().to_msg()
        composed.header.frame_id = first.header.frame_id
        composed.child_frame_id = second.child_frame_id

        # Получаем данные
        first_trans = [first.transform.translation.x,
                       first.transform.translation.y,
                       first.transform.translation.z]
        first_rot = [first.transform.rotation.x,
                     first.transform.rotation.y,
                     first.transform.rotation.z,
                     first.transform.rotation.w]

        second_trans = [second.transform.translation.x,
                        second.transform.translation.y,
                        second.transform.translation.z]
        second_rot = [second.transform.rotation.x,
                      second.transform.rotation.y,
                      second.transform.rotation.z,
                      second.transform.rotation.w]

        # Применяем вращение и трансляцию
        new_trans = tf.quaternion_multiply(tf.quaternion_multiply(first_rot, second_trans + [0]), tf.quaternion_conjugate(first_rot))[:3]
        new_trans = [first_trans[i] + new_trans[i] for i in range(3)]
        new_rot = tf.quaternion_multiply(first_rot, second_rot)

        # Записываем в сообщение
        composed.transform.translation.x = new_trans[0]
        composed.transform.translation.y = new_trans[1]
        composed.transform.translation.z = new_trans[2]

        composed.transform.rotation.x = new_rot[0]
        composed.transform.rotation.y = new_rot[1]
        composed.transform.rotation.z = new_rot[2]
        composed.transform.rotation.w = new_rot[3]

        return composed




def main(args=None):
    rclpy.init(args=args)
    node = MapToOdomTF()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
