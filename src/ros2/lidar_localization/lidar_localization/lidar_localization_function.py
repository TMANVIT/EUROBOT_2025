import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import (
    Buffer,
    TransformListener,
    LookupException,
    ConnectivityException,
    ExtrapolationException,
)
from obstacle_detector.msg import Obstacles
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import numpy as np
from scipy.spatial.transform import Rotation as R
from collections import deque

NUM_LANDMARKS = 3


class LidarLocalization(Node):
    def __init__(self):
        super().__init__("lidar_localization_node")

        # Params
        self.declare_parameters(
            namespace="",
            parameters=[
                ("side", 0),
                ("debug_mode", False),
                ("visualize_candidate", True),
                ("likelihood_threshold", 0.001),
                ("consistency_threshold", 0.9),
                ("frame_id", "base_footprint"),
                ("robot_parent_frame_id", "map"),
                ("min_R", 0.001),  
                ("max_R", 0.1), 
                ("adaptation_window", 10),
            ],
        )
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
        self.parent_frame_id = (
            self.get_parameter("robot_parent_frame_id")
            .get_parameter_value()
            .string_value
        )
        self.min_R = self.get_parameter("min_R").get_parameter_value().double_value
        self.max_R = self.get_parameter("max_R").get_parameter_value().double_value
        self.adaptation_window = (
            self.get_parameter("adaptation_window").get_parameter_value().integer_value
        )

        self.initialize_variables()
        self.setup_communications()

        self.innovation_history = deque(
            maxlen=self.get_parameter("adaptation_window").value
        )

    def initialize_variables(self):
        self.robot_pose = np.zeros(3)  # [x, y, theta]
        self.P_pred = np.diag(
            [0.05**2, 0.05**2, 0.1]
        ) 
        self.R = np.diag([0.001, 0.001])
        self.newPose = False

        # Инициализация карты ориентиров
        self.initialize_landmarks_map()

    def setup_communications(self):
        self.lidar_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/lidar_pose", 10
        )

        if self.visualize_candidate:
            self.circles_pub = self.create_publisher(MarkerArray, "/candidates", 10)

        self.create_subscription(
            Obstacles, "/raw_obstacles", self.obstacle_callback, 10
        )
        self.create_subscription(
            PoseWithCovarianceStamped, "/bve_pose", self.pred_pose_callback, 10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def initialize_landmarks_map(self):
        self.landmarks_map = (
            [
                np.array([1.5122, 1.0041]),
                np.array([1.56546, -0.880723]),
                np.array([-1.6032, -0.014313]),
            ]
            if self.side == 0
            else [
                np.array([1.52048, 0.00173703]),
                np.array([-1.49866, -0.999966]),
                np.array([-1.49856, 0.994759]),
            ]
        )

        self.geometry_description_map = {}
        for i in range(3):
            for j in range(i + 1, 3):
                self.geometry_description_map[(i, j)] = np.linalg.norm(
                    self.landmarks_map[i] - self.landmarks_map[j]
                )

    def pred_pose_callback(self, msg):
        self.newPose = True

        quat = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ]
        position = msg.pose.pose.position
        orientation = R.from_quat(quat)

        self.robot_pose = np.array(
            [position.x, position.y, orientation.as_euler("xyz")[2]]
        )

        self.adapt_prediction_covariance(msg)

    def adapt_prediction_covariance(self, msg):
        cov = np.array(
            [
                [msg.pose.covariance[0], 0, 0],
                [0, msg.pose.covariance[7], 0],
                [0, 0, msg.pose.covariance[35]],
            ]
        )

        if len(self.innovation_history) > 0:
            avg_innovation = np.mean(np.array(self.innovation_history), axis=0)
            innovation_norm = np.linalg.norm(avg_innovation)

            adapt_factor = 1.0 + np.tanh(innovation_norm - 1.0)

            self.P_pred = cov * adapt_factor
        else:
            self.P_pred = cov

    def obstacle_callback(self, msg):
        if not self.newPose:
            self.get_logger().debug("No new pose prediction, skipping...")
            return

        # 1. Get robot pose on this timestamp 
        self.get_robot_pose(msg.header.stamp)

        # 2. Extract obstacles
        obstacles = [np.array([obs.center.x, obs.center.y]) for obs in msg.circles]

        # 3. Match obstacles with landmarks
        landmarks_as_obstacles = self.match_landmarks(obstacles)

        # 4. Create set of landmarks
        landmarks_set = self.get_landmarks_set(landmarks_as_obstacles)

        # 5. Calculate pose based on the best lanmarks set
        self.lidar_pose, self.lidar_cov = self.get_lidar_pose(landmarks_set)

        # 6. Adapt filter params
        self.adapt_filter_parameters()

        # 7. Publish results
        self.pose_publish(self.lidar_pose, self.lidar_cov)

    def adapt_filter_parameters(self):
        if hasattr(self, "lidar_pose") and hasattr(self, "robot_pose"):
            innovation = self.lidar_pose[:2] - self.robot_pose[:2]
            self.innovation_history.append(innovation)

            if len(self.innovation_history) >= self.adaptation_window:
                innovation_cov = np.cov(np.array(self.innovation_history).T)

                min_R = self.min_R
                max_R = self.max_R

                self.R = np.clip(innovation_cov, min_R, max_R)

                self.get_logger().debug(f"Adapted R matrix: {self.R}")

    def get_obs_candidate(self, landmark, obstacles):
        obs_candidates = []
        x_r, y_r, phi_r = self.robot_pose
        x_o, y_o = landmark

        r_prime = np.sqrt((x_o - x_r) ** 2 + (y_o - y_r) ** 2)
        theta_prime = angle_limit_checking(np.arctan2(y_o - y_r, x_o - x_r) - phi_r)

        H = np.array(
            [
                [-(x_o - x_r) / r_prime, -(y_o - y_r) / r_prime, 0],
                [(y_o - y_r) / r_prime**2, -(x_o - x_r) / r_prime**2, -1],
            ]
        )

        S = H @ self.P_pred @ H.T + self.R
        S_inv = np.linalg.inv(S)

        if self.visualize_candidate:
            marker_array = MarkerArray()

        for i, obs in enumerate(obstacles):
            r_z = np.sqrt(obs[0] ** 2 + obs[1] ** 2)
            theta_z = np.arctan2(obs[1], obs[0])

            y = np.array([r_z - r_prime, angle_limit_checking(theta_z - theta_prime)])

            di_square = y.T @ S_inv @ y
            likelihood = np.exp(-0.5 * di_square)

            if self.likelihood_threshold < likelihood <= 1.0:
                obs_candidates.append(
                    {"position": obs, "probability": likelihood, "innovation": y}
                )

                if self.visualize_candidate:
                    self.add_candidate_marker(marker_array, obs, likelihood, i)

        if self.visualize_candidate:
            self.circles_pub.publish(marker_array)

        return obs_candidates

    def add_candidate_marker(self, marker_array, obs, likelihood, idx):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "candidates"
        marker.id = idx * 2
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = obs[0]
        marker.pose.position.y = obs[1]
        marker.pose.position.z = 0.0
        marker.scale.x = marker.scale.y = 0.1
        marker.scale.z = 0.01
        marker.color = ColorRGBA(r=0.0, g=0.5, b=1.0, a=likelihood)
        marker_array.markers.append(marker)

        # Текстовый маркер с вероятностью
        text_marker = Marker()
        text_marker.header = marker.header
        text_marker.ns = "text"
        text_marker.id = idx * 2 + 1
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = obs[0]
        text_marker.pose.position.y = obs[1]
        text_marker.pose.position.z = 0.1
        text_marker.scale.z = 0.1
        text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        text_marker.text = f"{likelihood:.2f}"
        marker_array.markers.append(text_marker)

    def pose_publish(self, lidar_pose, lidar_cov):
        self.lidar_pose_msg = PoseWithCovarianceStamped()
        self.lidar_pose_msg.header.stamp = self.get_clock().now().to_msg()
        self.lidar_pose_msg.header.frame_id = self.parent_frame_id
        self.lidar_pose_msg.pose.pose.position.x = lidar_pose[0]
        self.lidar_pose_msg.pose.pose.position.y = lidar_pose[1]
        self.lidar_pose_msg.pose.pose.position.z = 0.0
        self.lidar_pose_msg.pose.pose.orientation.x = 0.0
        self.lidar_pose_msg.pose.pose.orientation.y = 0.0
        self.lidar_pose_msg.pose.pose.orientation.z = np.sin(lidar_pose[2] / 2)
        self.lidar_pose_msg.pose.pose.orientation.w = np.cos(lidar_pose[2] / 2)
        self.lidar_pose_msg.pose.covariance = [
            lidar_cov[0, 0],
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            lidar_cov[1, 1],
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            lidar_cov[2, 2],
        ]
        self.get_logger().debug(f"lidar_pose: {lidar_pose}")
        self.lidar_pose_pub.publish(self.lidar_pose_msg)

    def get_lidar_pose(self, landmarks_set):
        # Initialize default values
        lidar_pose = self.robot_pose
        lidar_cov = self.P_pred

        if not landmarks_set:
            self.get_logger().debug("landmarks_set is empty")
            return lidar_pose, lidar_cov

        # Prefer the set with more beacons
        landmarks_set = sorted(
            landmarks_set, key=lambda x: len(x["beacons"]), reverse=True
        )

        # With the most beacons possible, prefer the set with the highest probability_set
        max_likelihood = max(set["probability_set"] for set in landmarks_set)
        max_likelihood_idx = next(
            i
            for i, set in enumerate(landmarks_set)
            if set["probability_set"] == max_likelihood
        )

        # If the most likely set has at least 3 beacons
        if len(landmarks_set[max_likelihood_idx]["beacons"]) >= 3:
            beacons = [
                landmarks_set[max_likelihood_idx]["beacons"][i] for i in range(3)
            ]
            A = np.zeros((2, 2))
            b = np.zeros(2)
            dist_beacon_robot = [np.linalg.norm(beacon) for beacon in beacons]

            A[0, 0] = 2 * (self.landmarks_map[0][0] - self.landmarks_map[2][0])
            A[0, 1] = 2 * (self.landmarks_map[0][1] - self.landmarks_map[2][1])
            A[1, 0] = 2 * (self.landmarks_map[1][0] - self.landmarks_map[2][0])
            A[1, 1] = 2 * (self.landmarks_map[1][1] - self.landmarks_map[2][1])

            b[0] = (
                (self.landmarks_map[0][0] ** 2 - self.landmarks_map[2][0] ** 2)
                + (self.landmarks_map[0][1] ** 2 - self.landmarks_map[2][1] ** 2)
                + (dist_beacon_robot[2] ** 2 - dist_beacon_robot[0] ** 2)
            )
            b[1] = (
                (self.landmarks_map[1][0] ** 2 - self.landmarks_map[2][0] ** 2)
                + (self.landmarks_map[1][1] ** 2 - self.landmarks_map[2][1] ** 2)
                + (dist_beacon_robot[2] ** 2 - dist_beacon_robot[1] ** 2)
            )

            try:
                X = np.linalg.solve(A.T @ A, A.T @ b)
                if X[0] < -1.5 or X[0] > 1.5 or X[1] < -1 or X[1] > 1:
                    self.get_logger().warn("Estimated position out of bounds")
                    return lidar_pose, lidar_cov  # Return default values

                lidar_pose[0] = X[0]
                lidar_pose[1] = X[1]

                robot_sin = 0
                robot_cos = 0

                for i in range(3):
                    theta = angle_limit_checking(
                        np.arctan2(
                            self.landmarks_map[i][1] - lidar_pose[1],
                            self.landmarks_map[i][0] - lidar_pose[0],
                        )
                        - np.arctan2(beacons[i][1], beacons[i][0])
                    )
                    robot_sin += np.sin(theta)
                    robot_cos += np.cos(theta)

                lidar_pose[2] = angle_limit_checking(np.arctan2(robot_sin, robot_cos))

                # lidar_pose = self.pose_compensation(lidar_pose)

                lidar_cov[0, 0] /= max_likelihood
                lidar_cov[1, 1] /= max_likelihood
                lidar_cov[2, 2] /= max_likelihood

            except np.linalg.LinAlgError as e:
                self.get_logger().warn(f"Linear algebra error: {e}")
                return lidar_pose, lidar_cov  # Return default values

        else:
            self.get_logger().info("Not enough beacons")
            return lidar_pose, lidar_cov  # Return default values

        return lidar_pose, lidar_cov

    def get_landmarks_set(self, landmarks_candidate):
        landmarks_set = []
        for i in range(len(landmarks_candidate[0]["obs_candidates"])):
            for j in range(len(landmarks_candidate[1]["obs_candidates"])):
                for k in range(len(landmarks_candidate[2]["obs_candidates"])):
                    set = {
                        "beacons": {
                            0: landmarks_candidate[0]["obs_candidates"][i]["position"],
                            1: landmarks_candidate[1]["obs_candidates"][j]["position"],
                            2: landmarks_candidate[2]["obs_candidates"][k]["position"],
                        }
                    }
                    # consistency of the set
                    set["consistency"] = self.get_geometry_consistency(set["beacons"])
                    if set["consistency"] < self.consistency_threshold:
                        self.get_logger().debug(
                            f"Geometry consistency is less than {self.consistency_threshold}: {set['consistency']}"
                        )
                        continue
                    # probability of the set
                    set["probability_set"] = (
                        landmarks_candidate[0]["obs_candidates"][i]["probability"]
                        * landmarks_candidate[1]["obs_candidates"][j]["probability"]
                        * landmarks_candidate[2]["obs_candidates"][k]["probability"]
                    )
                    landmarks_set.append(set)

        # print landmarks_set for debug
        if self.debug_mode:
            for i, set in enumerate(landmarks_set):
                print(f"Set {i + 1}:")
                print(f"Probability: {set['probability_set']}")
                print(f"Geometry Consistency: {set['consistency']}")

        return landmarks_set

    def match_landmarks(self, obstacles):
        landmarks_candidate = []
        for landmark in self.landmarks_map:
            candidate = {
                "landmark": landmark,
                "obs_candidates": self.get_obs_candidate(landmark, obstacles),
            }
            landmarks_candidate.append(candidate)

        # print landmarks_candidate for debug
        if self.debug_mode:
            for i, landmark in enumerate(landmarks_candidate):
                self.get_logger().info(f"Landmark {i + 1}: {landmark['landmark']}")
                for j, obs_candidate in enumerate(landmark["obs_candidates"]):
                    self.get_logger().info(
                        f"Obs {j + 1}: {obs_candidate['position']} with probability {obs_candidate['probability']}"
                    )
        return landmarks_candidate

    def init_landmarks_map(self):
        geometry_description_map = {}
        for i in range(NUM_LANDMARKS):
            for j in range(i + 1, NUM_LANDMARKS):
                if i == j:
                    continue
                d_ij = np.linalg.norm(self.landmarks_map[i] - self.landmarks_map[j])
                geometry_description_map[(i, j)] = d_ij
        return geometry_description_map

    def get_robot_pose(self, time):
        try:
            self.predict_transform = self.tf_buffer.lookup_transform(
                self.parent_frame_id, self.frame_id, time
            )
            euler = R.from_quat(
                [
                    self.predict_transform.transform.rotation.x,
                    self.predict_transform.transform.rotation.y,
                    self.predict_transform.transform.rotation.z,
                    self.predict_transform.transform.rotation.w,
                ]
            ).as_euler(seq="xyz", degrees=False)[2]
            self.robot_pose = np.array(
                [
                    self.predict_transform.transform.translation.x,
                    self.predict_transform.transform.translation.y,
                    euler,
                ]
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(
                f"Could not transform {self.parent_frame_id} to {self.frame_id}: {e}"
            )
            self.get_logger().debug("now try to use the latest topic")

    def get_geometry_consistency(self, beacons):
        geometry_description = {}
        consistency = 1.0

        # lenB can be 2, 3 or 4
        # use the index of the beacons to calculate the distance between them
        for i in beacons:
            for j in beacons:
                if i == j:
                    continue
                geometry_description[(i, j)] = np.linalg.norm(beacons[i] - beacons[j])
                # self.get_logger().debug(f"Beacon {i} to Beacon {j} # TODO: compensationdistance: {geometry_description[(i, j)]}")
                if (i, j) in self.geometry_description_map:
                    expected_distance = self.geometry_description_map[(i, j)]
                    consistency *= (
                        1
                        - np.abs(geometry_description[(i, j)] - expected_distance)
                        / expected_distance
                    )
                # if the index is not found in map, it is probably on the lower triangle of the matrix

        if self.side == 0:
            if np.cross(beacons[1] - beacons[0], beacons[2] - beacons[0]) > 0:
                consistency = 0
        elif self.side == 1:
            if np.cross(beacons[1] - beacons[0], beacons[2] - beacons[0]) < 0:
                consistency = 0

        return consistency

    def pose_compensation(self, lidar_pose):
        # find the translation and rotation from obs_time to now using TF
        try:
            now_transform = self.tf_buffer.lookup_transform(  # get the latest transform
                self.parent_frame_id,
                self.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
            relative_transform = now_transform
            relative_transform.transform.translation.x -= (
                self.predict_transform.transform.translation.x
            )
            relative_transform.transform.translation.y -= (
                self.predict_transform.transform.translation.y
            )
            # find the relative rotation
            q1_inv = [
                self.predict_transform.transform.rotation.x,
                self.predict_transform.transform.rotation.y,
                self.predict_transform.transform.rotation.z,
                -self.predict_transform.transform.rotation.w,  # Negate for inverse
            ]

            q0 = now_transform.transform.rotation
            qr = [
                q0.x * q1_inv[3]
                + q0.w * q1_inv[0]
                + q0.y * q1_inv[2]
                - q0.z * q1_inv[1],
                q0.y * q1_inv[3]
                + q0.w * q1_inv[1]
                + q0.z * q1_inv[0]
                - q0.x * q1_inv[2],
                q0.z * q1_inv[3]
                + q0.w * q1_inv[2]
                + q0.x * q1_inv[1]
                - q0.y * q1_inv[0],
                q0.w * q1_inv[3]
                - q0.x * q1_inv[0]
                - q0.y * q1_inv[1]
                - q0.z * q1_inv[2],
            ]

            lidar_pose[0] += relative_transform.transform.translation.x
            lidar_pose[1] += relative_transform.transform.translation.y
            lidar_pose[2] += R.from_quat([qr[0], qr[1], qr[2], qr[3]]).as_euler(
                seq="xyz", degrees=False
            )[2]
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(
                f"Could not transform {self.parent_frame_id} to {self.frame_id}: {e}"
            )
            self.get_logger().error("Could not transform the robot pose")
        return lidar_pose


def angle_limit_checking(theta):
    """Нормализация угла в диапазон [-π, π]"""
    return (theta + np.pi) % (2 * np.pi) - np.pi


def main(args=None):
    rclpy.init(args=args)
    node = LidarLocalization()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
