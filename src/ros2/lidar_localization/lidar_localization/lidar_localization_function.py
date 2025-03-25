import rclpy
from rclpy.node import Node
import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseWithCovarianceStamped, Point
from obstacle_detector.msg import Obstacles
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from itertools import combinations
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from sklearn.linear_model import RANSACRegressor  # Для RANSAC

class LidarLocalization(Node):
    def __init__(self):
        super().__init__("lidar_localization_node")
        
        # Параметры
        self.declare_parameter("side", 0)
        self.declare_parameter("debug_mode", False)
        self.declare_parameter("ransac_threshold", 1000)  # Порог для RANSAC
        self.declare_parameter("min_landmarks", 3)  # Минимальное число ориентиров для локализации
        self.declare_parameter("frame_id", "lidar_link")
        self.declare_parameter("robot_parent_frame_id", "base_footprint")

        self.side = self.get_parameter("side").value
        self.debug_mode = self.get_parameter("debug_mode").value
        self.ransac_threshold = self.get_parameter("ransac_threshold").value
        self.min_landmarks = self.get_parameter("min_landmarks").value
        self.frame_id = self.get_parameter("frame_id").value
        self.parent_frame_id = self.get_parameter("robot_parent_frame_id").value

        # Карта ориентиров (пример для side=0)
        # self.landmarks_map = np.array([
        #     [1.56289, 0.993082],
        #     [1.54, -0.92],
        #     [-1.5499, -0.0027658]
        # ]) if self.side == 0 else np.array([
        #     [1.52048, 0.00173703],
        #     [-1.49866, -0.999966],
        #     [-1.49856, 0.994759]
        # ])
        self.landmarks_map = np.array([
            [0.99, -1.56],
            [-0.92, -1.54],
            [-0.0027658, 1.54]])

        # Публикаторы и подписчики
        self.lidar_pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/lidar_pose", 10)
        self.markers_pub = self.create_publisher(MarkerArray, "/landmark_matches", 10)
        self.create_subscription(Obstacles, "/raw_obstacles", self.obstacle_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/pred_pose", self.pred_pose_callback, 10)

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Фильтр Калмана (упрощённая реализация)
        self.robot_pose = np.zeros(3)  # [x, y, yaw]
        self.robot_cov = np.diag([0.1, 0.1, 0.05])  # Начальная ковариация

        # Кэш последних измерений
        self.last_obstacles = None
        self.last_pose_time = None

    def pred_pose_callback(self, msg):
        """Обновление предсказанного положения робота (odometry)."""
        quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        yaw = R.from_quat(quat).as_euler('xyz')[2]
        self.robot_pose = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, yaw])
        self.robot_cov = np.diag([msg.pose.covariance[0], msg.pose.covariance[7], msg.pose.covariance[35]])

    def obstacle_callback(self, msg):
        """Обработка препятствий от лидара."""
        if not hasattr(self, 'robot_pose'):
            return

        obstacles = np.array([[obs.center.x, obs.center.y] for obs in msg.circles])
        if len(obstacles) < self.min_landmarks:
            return

        # Шаг 1: Найти соответствия между препятствиями и картой с помощью RANSAC
        matched_landmarks, matched_obstacles = self.match_landmarks_ransac(obstacles)
        self.get_logger().info(f"{matched_landmarks}, {matched_obstacles}")
        

        if len(matched_landmarks) < self.min_landmarks:
            self.get_logger().warn("Недостаточно соответствий для локализации!")
            return

        # Шаг 2: Оценить положение робота через метод наименьших квадратов
        estimated_pose, pose_cov = self.estimate_pose(matched_landmarks, matched_obstacles)

        # Шаг 3: Обновить фильтр Калмана
        self.update_kalman_filter(estimated_pose, pose_cov)

        # Визуализация
        self.visualize_matches(matched_landmarks, matched_obstacles)

    def match_landmarks_ransac(self, obstacles):
        """Находит соответствия между препятствиями и картой с помощью RANSAC.
        
        Возвращает:
            tuple: (matched_landmarks, matched_obstacles) — массивы shape=(N, 2).
                Если соответствий нет, возвращает ([], []).
        """
        if len(obstacles) < self.min_landmarks:
            return [], []

        # Преобразуем препятствия в полярные координаты (r, θ)
        obstacles_polar = np.array([
            [np.sqrt(obs[0]**2 + obs[1]**2), np.arctan2(obs[1], obs[0])] 
            for obs in obstacles
        ])

        # Инициализируем RANSAC
        model = RANSACRegressor(
            min_samples=self.min_landmarks,
            residual_threshold=self.ransac_threshold,
            max_trials=100  # Ограничиваем число итераций
        )

        # Попробуем сопоставить с каждым подмножеством карты
        best_inliers = []
        best_landmarks = []
        
        for landmark_ids in combinations(range(len(self.landmarks_map)), self.min_landmarks):
            landmarks_subset = self.landmarks_map[list(landmark_ids)]
            
            try:
                # Фитируем RANSAC: предсказываем ориентиры по препятствиям
                model.fit(obstacles, landmarks_subset)
                if not hasattr(model, 'inlier_mask_'):
                    continue
                    
                inliers = model.inlier_mask_
                if np.sum(inliers) > len(best_inliers):
                    best_inliers = inliers
                    best_landmarks = landmarks_subset

            except Exception as e:
                self.get_logger().warn(f"RANSAC error: {e}", throttle_duration_sec=5)

        if len(best_inliers) == 0:
            return [], []

        # Возвращаем только те препятствия, которые попали inliers
        matched_obstacles = obstacles[best_inliers]
        matched_landmarks = best_landmarks
        
        return matched_landmarks, matched_obstacles

    def estimate_pose(self, landmarks, obstacles):
        """Оценивает положение робота через метод наименьших квадратов."""
        A = []
        b = []
        
        for (x_map, y_map), (x_obs, y_obs) in zip(landmarks, obstacles):
            A.append([1, 0, -y_obs])
            A.append([0, 1, x_obs])
            b.append(x_map - x_obs)
            b.append(y_map - y_obs)

        A = np.array(A)
        b = np.array(b)

        # Решение системы Ax = b
        x = np.linalg.lstsq(A, b, rcond=None)[0]
        estimated_pose = np.array([x[0], x[1], np.arctan2(x[2], 1)])  # [x, y, yaw]

        # Упрощённый расчёт ковариации (можно заменить на более точный)
        pose_cov = np.diag([0.05, 0.05, 0.02])  # Зависит от точности RANSAC

        return estimated_pose, pose_cov

    def update_kalman_filter(self, estimated_pose, pose_cov):
        """Обновляет оценку положения робота с помощью фильтра Калмана."""
        # Предсказание (используем odometry)
        K = np.eye(3)  # Упрощённый коэффициент Калмана

        # Коррекция
        self.robot_pose += K @ (estimated_pose - self.robot_pose)
        self.robot_cov = (np.eye(3) - K) @ self.robot_cov

        # Публикация результата
        self.publish_pose()

    def publish_pose(self):
        """Публикует оценённое положение робота."""
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.parent_frame_id
        msg.pose.pose.position.x = self.robot_pose[0]
        msg.pose.pose.position.y = self.robot_pose[1]
        msg.pose.pose.orientation = R.from_euler('z', self.robot_pose[2]).as_quat()
        msg.pose.covariance = list(np.diag(self.robot_cov))
        self.lidar_pose_pub.publish(msg)

    def visualize_matches(self, landmarks, obstacles):
        """Визуализирует соответствия между препятствиями и картой."""
        marker_array = MarkerArray()

        for i, (landmark, obs) in enumerate(zip(landmarks, obstacles)):
            # Маркер для ориентира на карте
            marker = Marker()
            marker.header.frame_id = "map"
            marker.id = i * 2
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = Point(x=landmark[0], y=landmark[1], z=0.0)
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
            marker_array.markers.append(marker)

            # Маркер для препятствия от лидара
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.id = i * 2 + 1
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = Point(x=obs[0], y=obs[1], z=0.0)
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
            marker_array.markers.append(marker)

        self.markers_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = LidarLocalization()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()