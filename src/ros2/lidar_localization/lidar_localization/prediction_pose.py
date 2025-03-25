import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class PredPublisher(Node):
    def __init__(self):
        super().__init__("pred_publisher")
        self.pose_pred = PoseWithCovarianceStamped()
        self.pose_pred.header.frame_id = "map"
        self.pose_pred.pose.pose.position.x = 0.0
        self.pose_pred.pose.pose.position.y = 0.0
        self.pose_pred.pose.pose.position.z = 0.0
        self.pose_pred.pose.pose.orientation.x = 0.0
        self.pose_pred.pose.pose.orientation.y = 0.0
        self.pose_pred.pose.pose.orientation.z = 0.0
        self.pose_pred.pose.pose.orientation.w = 1.0
        self.pose_pred.pose.covariance = [0.0] * 36
        self.pose_pred.pose.covariance[0] = 0.0025
        self.pose_pred.pose.covariance[7] = 0.0025
        self.pose_pred.pose.covariance[35] = 0.25
        self.init_pose_detected = False

        self.init_pose = self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self.initpose_callback, 10
        )
        self.pub = self.create_publisher(PoseWithCovarianceStamped, "/pred_pose", 10)
        # self.sub = self.create_subscription(
        #     PoseWithCovarianceStamped, "/lidar_pose", self.param_callback, 10
        # )

        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(0.01, self.publish_pose)  # 100Hz

    # def param_callback(self, msg):
    #     # Modify the pose_pred data based on the received parameter
    #     self.pose_pred.pose.pose.position.x = msg.pose.pose.position.x
    #     self.pose_pred.pose.pose.position.y = msg.pose.pose.position.y
    #     self.pose_pred.pose.pose.position.z = msg.pose.pose.position.z
    #     self.pose_pred.pose.pose.orientation.x = msg.pose.pose.orientation.x
    #     self.pose_pred.pose.pose.orientation.y = msg.pose.pose.orientation.y
    #     self.pose_pred.pose.pose.orientation.z = msg.pose.pose.orientation.z
    #     self.pose_pred.pose.pose.orientation.w = msg.pose.pose.orientation.w
    #     self.pose_pred.pose.covariance = msg.pose.covariance

    #     # Broadcast static transform from map to robot base
    #     t = TransformStamped()
    #     t.header.stamp = self.get_clock().now().to_msg()
    #     t.header.frame_id = "map"
    #     t.child_frame_id = "lidar_predict"
    #     t.transform.translation.x = self.pose_pred.pose.pose.position.x
    #     t.transform.translation.y = self.pose_pred.pose.pose.position.y
    #     t.transform.translation.z = self.pose_pred.pose.pose.position.z
    #     t.transform.rotation = self.pose_pred.pose.pose.orientation

    #     self.br.sendTransform(t)

    def publish_pose(self):
        self.pose_pred.header.stamp = self.get_clock().now().to_msg()
        if self.init_pose_detected:
            self.pub.publish(self.pose_pred)

    def initpose_callback(self, msg):
        if not self.init_pose_detected:
            self.pose_pred.pose.pose.position.x = msg.pose.pose.position.x
            self.pose_pred.pose.pose.position.y = msg.pose.pose.position.y
            self.pose_pred.pose.pose.position.z = msg.pose.pose.position.z
            self.pose_pred.pose.pose.orientation.x = msg.pose.pose.orientation.x
            self.pose_pred.pose.pose.orientation.y = msg.pose.pose.orientation.y
            self.pose_pred.pose.pose.orientation.z = msg.pose.pose.orientation.z
            self.pose_pred.pose.pose.orientation.w = msg.pose.pose.orientation.w
            self.init_pose_detected = True


def main(args=None):
    rclpy.init(args=args)
    pred_publisher = PredPublisher()
    rclpy.spin(pred_publisher)
    pred_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
