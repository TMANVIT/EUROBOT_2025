import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from obstacle_detector.msg import Obstacles
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import numpy as np
from scipy.spatial.transform import Rotation as R 
from tf2_ros import TransformException, Buffer, TransformListener

NUM_LANDMARKS = 3  # Константа для количества ориентиров

class LidarLocalization(Node):
    """Класс для локализации с использованием LIDAR в ROS2 с учетом препятствий в lidar_link"""

    def __init__(self):
        super().__init__('lidar_localization_node')

        # Declare parameters
        self.declare_parameter("side", 0)
        self.declare_parameter("debug_mode", False)
        self.declare_parameter("visualize_candidate", True)
        self.declare_parameter("likelihood_threshold", 0.001)
        self.declare_parameter("consistency_threshold", 0.9)
        self.declare_parameter("frame_id", "lidar_link")
        self.declare_parameter('robot_parent_frame_id', 'base_footprint')

        # Get parameters
        self.side = self.get_parameter("side").get_parameter_value().integer_value
        self.debug_mode = (
            self.get_parameter("debug_mode").get_parameter_value().bool_value
        )
        self.visualize_candidate = (
            self.get_parameter("visualize_candidate").get_parameter_value().bool_value
        )
        self.likelihood_threshold = (
            self.get_parameter("likelihood_threshold")
            .get_parameter_value()
            .double_value
        )
        self.consistency_threshold = (
            self.get_parameter("consistency_threshold")
            .get_parameter_value()
            .double_value
        )
        self.frame_id = (
            self.get_parameter("frame_id").get_parameter_value().string_value
        )
        self.parent_frame_id = self.get_parameter('robot_parent_frame_id').get_parameter_value().string_value

        # Инициализация карты ориентиров
        if self.side == 0:
            self.landmarks_map = [
                np.array([1.56289, 0.993082]),
                np.array([1.5617, -0.999224]),
                np.array([-1.5099, -0.0087658])
            ]
        elif self.side == 1:
            self.landmarks_map = [
                np.array([1.52048, 0.00173703]),
                np.array([-1.49866, -0.999966]),
                np.array([-1.49856, 0.994759])
            ]

        self.lidar_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/lidar_pose", 10
        )
        if self.visualize_candidate:
            self.circles_pub = self.create_publisher(MarkerArray, "/candidates", 10)
        self.subscription = self.create_subscription(
            Obstacles, "/raw_obstacles", self.obstacle_callback, 10
        )
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped, "/pred_pose", self.pred_pose_callback, 10
        )
        self.subscription  # prevent unused variable warning
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.geometry_description_map = self.init_landmarks_map()
        self.robot_pose = []
        self.P_pred = np.array([[0.05**2, 0.0, 0.0], [0.0, 0.05**2, 0.0], [0.0, 0.0, 0.1]])
        self.newPose = False
        self.R = np.array(
            [[0.001, 0.0], [0.0, 0.001]]
        )
        self.lidar_pose_msg = PoseWithCovarianceStamped()

    def obstacle_callback(self, msg):
        """Обработка данных от препятствий с преобразованием из lidar_link в map"""
        if not self.init_pose_detected or self.robot_pose is None or self.P_pred is None:
            self.get_logger().debug("Skipping obstacle callback: initial pose or pred_pose not received")
            return

        self.get_logger().debug('Obstacle detected')

        # Попытка получить трансформацию из lidar_link в map
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',          # Целевая система координат
                'lidar_link',   # Исходная система координат
                msg.header.stamp,
                rclpy.duration.Duration(seconds=0.1)  # Тайм-аут ожидания
            )
        except TransformException as e:
            self.get_logger().warn(f"Could not transform obstacles from lidar_link to map: {e}")
            return

        # Преобразование координат препятствий из lidar_link в map
        self.obs_raw = []
        for obs in msg.circles:
            self.obs_raw.append(np.array([obs.center.x, obs.center.y]))
            self.get_logger().info(f"x = {obs.center.x}; y = {obs.center.y}")
        self.get_logger().info(f"complete")
        self.obs_time = msg.header.stamp
        # data processing
        try:
            self.predict_transform = self.tf_buffer.lookup_transform(
                self.parent_frame_id,
                self.frame_id,
                self.obs_time
            )
            euler = R.from_quat(
                    [self.predict_transform.transform.rotation.x,
                    self.predict_transform.transform.rotation.y,
                    self.predict_transform.transform.rotation.z,
                    self.predict_transform.transform.rotation.w]).as_euler(seq="xyz", degrees=False)[2]
            self.robot_pose = np.array([
                self.predict_transform.transform.translation.x,
                self.predict_transform.transform.translation.y,
                euler                
            ])
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f'Could not transform {self.parent_frame_id} to {self.frame_id}: {e}')
            self.get_logger().debug("now try to use the latest topic")
            if self.newPose == False: 
                self.get_logger().error("no new predict topic, skip.")
                return
        # self.get_logger().debug(f"New Pose detected x = {self.robot_pose[0]}, y = {self.robot_pose[1]}, yaw = {self.robot_pose[2]}")
        self.landmarks_candidate = self.get_landmarks_candidate()
        self.landmarks_set = self.get_landmarks_set()
        
        if not self.landmarks_set:
            self.get_logger().debug("Empty landmarks set")
            return

        self.lidar_pose, self.lidar_cov = self.get_lidar_pose()
        self.clear_data()

    def transform_point(self, point, transform):
        """Преобразование точки из одной системы координат в другую с использованием TF"""
        # Извлечение трансляции и вращения из transform
        trans = transform.transform.translation
        rot = transform.transform.rotation
        rot_matrix = R.from_quat([rot.x, rot.y, rot.z, rot.w]).as_matrix()[:2, :2]  # Только 2D вращение

        # Преобразование координат
        point_local = np.array([point.x, point.y])
        point_global = rot_matrix @ point_local + np.array([trans.x, trans.y])

        # Создание новой точки в системе map
        point_map = Point()
        point_map.x = point_global[0]
        point_map.y = point_global[1]
        point_map.z = 0.0  # Предполагаем плоское движение
        return point_map

    def pred_pose_callback(self, msg):
        """Обработка предсказанной позы робота"""
        if not self.init_pose_detected:
            self.get_logger().debug("Skipping pred_pose update: initial pose not received")
            return

        self.get_logger().debug("Robot pose callback triggered")
        self.newPose = True
        quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        yaw = R.from_quat(quat).as_euler(seq='xyz', degrees=False)[2]

        if yaw < 0:
            yaw += 2 * np.pi

        self.robot_pose = np.array(
            [msg.pose.pose.position.x, msg.pose.pose.position.y, yaw]
        )
        self.P_pred = np.array(
            [
                [msg.pose.covariance[0] * 100, 0, 0],
                [0, msg.pose.covariance[7] * 100, 0],
                [0, 0, msg.pose.covariance[35] * 1e6],
            ]
        )

    def init_landmarks_map(self):
        """Инициализация геометрического описания карты ориентиров"""
        geometry_description_map = {}
        for i in range(NUM_LANDMARKS):
            for j in range(i + 1, NUM_LANDMARKS):
                d_ij = np.linalg.norm(self.landmarks_map[i] - self.landmarks_map[j])
                geometry_description_map[(i, j)] = d_ij
        return geometry_description_map

    def clear_data(self):
        """Очистка временных данных"""
        self.obs_raw = []
        self.robot_pose = None
        self.P_pred = None
        self.landmarks_candidate = []
        self.landmarks_set = []
        self.newPose = False
        self.predict_transform = None

    def get_obs_candidate(self, landmark):
        """Получение кандидатов наблюдений для конкретного ориентира"""
        obs_candidates = []
        x_r, y_r, phi_r = self.robot_pose
        x_o, y_o = landmark
        
        r_prime = np.sqrt((x_o - x_r) ** 2 + (y_o - y_r) ** 2)
        theta_prime = angle_limit_checking(np.arctan2(y_o - y_r, x_o - x_r) - phi_r)

        H = np.array([
            [-(x_o - x_r) / r_prime, -(y_o - y_r) / r_prime, 0],
            [(y_o - y_r) / r_prime ** 2, -(x_o - x_r) / r_prime ** 2, -1]
        ])
        
        S = H @ self.P_pred @ H.T + self.R
        S_inv = np.linalg.inv(S)
        S_det = np.linalg.det(S)
        normalizer = 1 / np.sqrt((2 * np.pi) ** 2 * S_det)

        marker_array = MarkerArray()
        marker_id = 0

        for obs in self.obs_raw:
            r_z = np.sqrt(obs[0] ** 2 + obs[1] ** 2)
            theta_z = np.arctan2(obs[1], obs[0])
            y = np.array([r_z - r_prime, angle_limit_checking(theta_z - theta_prime)])
            di_square = y.T @ S_inv @ y
            likelihood = normalizer * np.exp(-0.5 * di_square)
            likelihood = likelihood / normalizer

            if likelihood > self.likelihood_threshold:
                obs_candidates.append({"position": obs, "probability": likelihood})
                if self.visualize_candidate:
                    marker = Marker()
                    marker.header.frame_id = self.frame_id
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.ns = "candidates"
                    marker.type = Marker.SPHERE
                    marker.action = Marker.ADD
                    marker.scale.x = 0.1
                    marker.scale.y = 0.1
                    marker.scale.z = 0.01

                    text_marker = Marker()
                    text_marker.header.frame_id = self.frame_id
                    text_marker.header.stamp = self.get_clock().now().to_msg()
                    text_marker.ns = "text"
                    text_marker.type = Marker.TEXT_VIEW_FACING
                    text_marker.action = Marker.ADD
                    text_marker.scale.z = 0.1
                    text_marker.color = ColorRGBA(
                        r=1.0, g=1.0, b=1.0, a=1.0
                    )  # White text

                    # use visualization_msgs to visualize the likelihood
                    marker.pose.position.x = obs[0]
                    marker.pose.position.y = obs[1]
                    marker.pose.position.z = 0.0
                    marker.color = ColorRGBA(r=0.0, g=0.5, b=1.0, a=likelihood)
                    marker_id += 1
                    marker.id = marker_id
                    marker_array.markers.append(marker)
                    text_marker.pose.position.x = obs[0]
                    text_marker.pose.position.y = obs[1]
                    text_marker.pose.position.z = 0.1
                    text_marker.text = f"{likelihood:.2f}"
                    text_marker.id = marker_id
                    marker_array.markers.append(text_marker)
        if self.visualize_candidate:
            self.circles_pub.publish(marker_array)
            self.get_logger().debug("Published marker array")
            # clean up
            marker_array.markers.clear()

        return obs_candidates

    def get_landmarks_candidate(self):
        """Получение списка кандидатов для всех ориентиров"""
        landmarks_candidate = []
        for landmark in self.landmarks_map:
            candidate = {
                'landmark': landmark,
                'obs_candidates': self.get_obs_candidate(landmark)
            }
            landmarks_candidate.append(candidate)

        if self.debug_mode:
            for i, landmark in enumerate(landmarks_candidate):
                self.get_logger().info(f"Landmark {i + 1}: {landmark['landmark']}")
                for j, obs in enumerate(landmark['obs_candidates']):
                    self.get_logger().info(f"Obs {j + 1}: {obs['position']} with probability {obs['probability']}")
        
        return landmarks_candidate

    def get_landmarks_set(self):
        """Формирование наборов ориентиров"""
        landmarks_set = []
        
        for i in range(len(self.landmarks_candidate[0]['obs_candidates'])):
            for j in range(len(self.landmarks_candidate[1]['obs_candidates'])):
                for k in range(len(self.landmarks_candidate[2]['obs_candidates'])):
                    set_dict = {
                        'beacons': {
                            0: self.landmarks_candidate[0]['obs_candidates'][i]['position'],
                            1: self.landmarks_candidate[1]['obs_candidates'][j]['position'],
                            2: self.landmarks_candidate[2]['obs_candidates'][k]['position']
                        }
                    }
                    set_dict['consistency'] = self.get_geometry_consistency(set_dict['beacons'])
                    
                    if set_dict['consistency'] < self.consistency_threshold:
                        self.get_logger().debug(f"Geometry consistency {set_dict['consistency']} < {self.consistency_threshold}")
                        continue
                    
                    set_dict['probability_set'] = (self.landmarks_candidate[0]['obs_candidates'][i]['probability'] *
                                                 self.landmarks_candidate[1]['obs_candidates'][j]['probability'] *
                                                 self.landmarks_candidate[2]['obs_candidates'][k]['probability'])
                    landmarks_set.append(set_dict)

        if self.debug_mode:
            for i, set_dict in enumerate(landmarks_set):
                self.get_logger().info(f"Set {i + 1}: Probability={set_dict['probability_set']}, Consistency={set_dict['consistency']}")

        return landmarks_set

    def get_lidar_pose(self):
        """Вычисление позы LIDAR на основе наборов ориентиров"""
        if not self.landmarks_set:
            raise ValueError("landmarks_set is empty")

        self.landmarks_set.sort(key=lambda x: (len(x['beacons']), x['probability_set']), reverse=True)
        max_likelihood_idx = 0

        lidar_pose = np.zeros(3)
        lidar_cov = np.diag(
            [0.05**2, 0.05**2, 0.05**2]
        )
        self.get_logger().debug(f"landmarks_set: {self.landmarks_set}")
        # If the most likely set has at least 3 beacons
        if len(self.landmarks_set[max_likelihood_idx]["beacons"]) >= 3:
            beacons = [
                self.landmarks_set[max_likelihood_idx]["beacons"][i] for i in range(3)
            ]
            A = np.zeros((2, 2))
            b = np.zeros(2)
            dist_beacon_robot = [np.linalg.norm(beacon) for beacon in beacons]

            A = np.zeros((2, 2))
            b = np.zeros(2)
            A[0, 0] = 2 * (self.landmarks_map[0][0] - self.landmarks_map[2][0])
            A[0, 1] = 2 * (self.landmarks_map[0][1] - self.landmarks_map[2][1])
            A[1, 0] = 2 * (self.landmarks_map[1][0] - self.landmarks_map[2][0])
            A[1, 1] = 2 * (self.landmarks_map[1][1] - self.landmarks_map[2][1])
            b[0] = (self.landmarks_map[0][0]**2 - self.landmarks_map[2][0]**2 + 
                   self.landmarks_map[0][1]**2 - self.landmarks_map[2][1]**2 + 
                   dist_beacon_robot[2]**2 - dist_beacon_robot[0]**2)
            b[1] = (self.landmarks_map[1][0]**2 - self.landmarks_map[2][0]**2 + 
                   self.landmarks_map[1][1]**2 - self.landmarks_map[2][1]**2 + 
                   dist_beacon_robot[2]**2 - dist_beacon_robot[1]**2)

            try:
                X = np.linalg.solve(A.T @ A, A.T @ b)
                if X[0] < 0 or X[0] > 3 or X[1] < 0 or X[1] > 2:
                    return
                lidar_pose[0] = X[0]
                lidar_pose[1] = X[1]

                robot_sin = robot_cos = 0
                for i in range(3):
                    theta = angle_limit_checking(np.arctan2(self.landmarks_map[i][1] - lidar_pose[1], 
                                                          self.landmarks_map[i][0] - lidar_pose[0]) - 
                                               np.arctan2(beacons[i][1], beacons[i][0]))
                    robot_sin += np.sin(theta)
                    robot_cos += np.cos(theta)

                    lidar_pose[2] = angle_limit_checking(
                        np.arctan2(robot_sin, robot_cos)
                    )
                
                self.pose_compensation(lidar_pose)

                max_likelihood = self.landmarks_set[max_likelihood_idx]['probability_set']
                lidar_cov /= max_likelihood

                self.lidar_pose_msg.header.stamp = self.get_clock().now().to_msg()
                self.lidar_pose_msg.header.frame_id = 'map'
                self.lidar_pose_msg.pose.pose.position.x = lidar_pose[0]
                self.lidar_pose_msg.pose.pose.position.y = lidar_pose[1]
                self.lidar_pose_msg.pose.pose.position.z = 0.0
                quat = R.from_euler('z', lidar_pose[2]).as_quat()
                self.lidar_pose_msg.pose.pose.orientation.x = quat[0]
                self.lidar_pose_msg.pose.pose.orientation.y = quat[1]
                self.lidar_pose_msg.pose.pose.orientation.z = quat[2]
                self.lidar_pose_msg.pose.pose.orientation.w = quat[3]
                self.lidar_pose_msg.pose.covariance = lidar_cov.flatten().tolist() + [0.0] * 33
                self.lidar_pose_pub.publish(self.lidar_pose_msg)
                self.get_logger().debug(f"Published lidar_pose: {lidar_pose}")

            except np.linalg.LinAlgError as e:
                self.get_logger().warn(f"Linear algebra error: {e}")
        else:
            self.get_logger().info("Not enough beacons")

        return lidar_pose, lidar_cov

    def get_geometry_consistency(self, beacons):
        """Вычисление геометрической согласованности набора ориентиров"""
        geometry_description = {}
        consistency = 1.0

        for i in beacons:
            for j in beacons:
                if i >= j:
                    continue
                dist = np.linalg.norm(beacons[i] - beacons[j])
                geometry_description[(i, j)] = dist
                if (i, j) in self.geometry_description_map:
                    expected_distance = self.geometry_description_map[(i, j)]
                    consistency *= (
                        1
                        - np.abs(geometry_description[(i, j)] - expected_distance)
                        / expected_distance
                    )
                # if the index is not found in map, it is probably on the lower triangle of the matrix

        return consistency

    def pose_compensation(self, lidar_pose):
        # find the translation and rotation from obs_time to now using TF
        try:
            now_transform = self.tf_buffer.lookup_transform( # get the latest transform
                self.parent_frame_id,
                self.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            relative_transform = now_transform
            relative_transform.transform.translation.x -= self.predict_transform.transform.translation.x
            relative_transform.transform.translation.y -= self.predict_transform.transform.translation.y
            # find the relative rotation
            q1_inv = [
                self.predict_transform.transform.rotation.x,
                self.predict_transform.transform.rotation.y,
                self.predict_transform.transform.rotation.z,
                -self.predict_transform.transform.rotation.w  # Negate for inverse
            ]

            q0 = now_transform.transform.rotation
            qr = [
                q0.x * q1_inv[3] + q0.w * q1_inv[0] + q0.y * q1_inv[2] - q0.z * q1_inv[1],
                q0.y * q1_inv[3] + q0.w * q1_inv[1] + q0.z * q1_inv[0] - q0.x * q1_inv[2],
                q0.z * q1_inv[3] + q0.w * q1_inv[2] + q0.x * q1_inv[1] - q0.y * q1_inv[0],
                q0.w * q1_inv[3] - q0.x * q1_inv[0] - q0.y * q1_inv[1] - q0.z * q1_inv[2]
            ]
            
            lidar_pose[0] += relative_transform.transform.translation.x
            lidar_pose[1] += relative_transform.transform.translation.y
            lidar_pose[2] += R.from_quat([qr[0], qr[1], qr[2], qr[3]]).as_euler(seq='xyz', degrees=False)[2]
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f'Could not transform {self.robot_parent_frame_id} to {self.robot_frame_id}: {e}')
            self.get_logger().error("Could not transform the robot pose")
            return


def angle_limit_checking(theta):
    """Нормализация угла в диапазон [-π, π]"""
    while theta > np.pi:
        theta -= 2 * np.pi
    while theta <= -np.pi:
        theta += 2 * np.pi
    return theta

def main(args=None):
    """Главная функция для запуска узла"""
    rclpy.init(args=args)
    lidar_localization = LidarLocalization()
    rclpy.spin(lidar_localization)
    lidar_localization.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()