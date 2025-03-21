import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from tf2_ros import TransformBroadcaster


class PredPublisher(Node):
    """Узел для публикации предсказанной позы робота на основе данных LIDAR и начальной позы с камеры"""

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

        # Публикация предсказанной позы
        self.pub = self.create_publisher(PoseWithCovarianceStamped, "/pred_pose", 10)

        # Подписка на позу от LIDAR
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/lidar_pose",
            self.lidar_pose_callback,
            10
        )

        # Инициализация вещателя трансформаций
        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(0.01, self.publish_pose)  # Публикация на 100 Гц

        self.get_logger().info("PredPublisher node initialized, waiting for /initialpose")

    def lidar_pose_callback(self, msg):
        """Обновление предсказанной позы на основе данных от LIDAR"""
        if not self.init_pose_detected or self.pose_pred is None:
            self.get_logger().debug("Skipping LIDAR pose update: initial pose not received")
            return

        self.pose_pred.pose.pose.position.x = msg.pose.pose.position.x
        self.pose_pred.pose.pose.position.y = msg.pose.pose.position.y
        self.pose_pred.pose.pose.position.z = msg.pose.pose.position.z
        self.pose_pred.pose.pose.orientation = msg.pose.pose.orientation
        self.pose_pred.pose.covariance = msg.pose.covariance

        # Вещание трансформации от map к lidar_predict
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "lidar_predict"
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t)

    def publish_pose(self):
        """Публикация предсказанной позы с текущей меткой времени"""
        if not self.init_pose_detected or self.pose_pred is None:
            self.get_logger().debug("Skipping pose publication: initial pose not received")
            return

        self.pose_pred.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.pose_pred)

    def initpose_callback(self, msg):
        """Обработка начальной позы от внешней камеры"""
        if not self.init_pose_detected:
            # Инициализация pose_pred только после получения /initialpose
            self.pose_pred = PoseWithCovarianceStamped()
            self.pose_pred.header.frame_id = "map"
            self.pose_pred.pose.pose.position.x = msg.pose.pose.position.x
            self.pose_pred.pose.pose.position.y = msg.pose.pose.position.y
            self.pose_pred.pose.pose.position.z = msg.pose.pose.position.z
            self.pose_pred.pose.pose.orientation = msg.pose.pose.orientation
            self.pose_pred.pose.covariance = msg.pose.covariance  # Используем ковариацию от камеры
            self.init_pose_detected = True
            self.get_logger().info("Initial pose received from camera")


def main(args=None):
    """Главная функция для запуска узла"""
    rclpy.init(args=args)
    pred_publisher = PredPublisher()
    rclpy.spin(pred_publisher)
    pred_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()