import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped


class MapToOdomTF(Node):
    def __init__(self):
        super().__init__("map_to_odom_tf")

        self.create_subscription(
            PoseStamped, "/initialpose", self.initial_pose_callback, 10
        )
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def initial_pose_callback(self, msg):

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "odom"

        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z

        t.transform.rotation.x = msg.pose.orientation.x
        t.transform.rotation.y = msg.pose.orientation.y
        t.transform.rotation.z = msg.pose.orientation.z
        t.transform.rotation.w = msg.pose.orientation.w

        self.tf_broadcaster.sendTransform(t)

        initial_x = msg.pose.position.x
        initial_y = msg.pose.position.y

        initial_z = msg.pose.position.z
        self.get_logger().info(
            f"Initial pose set: x={initial_x}, y={initial_y}, z={initial_z}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = MapToOdomTF()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
