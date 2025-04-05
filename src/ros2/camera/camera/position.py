import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge
from cvFunctions.cvf import Camera  # Предполагается, что Camera находится в cvFunctions.cvf
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
import numpy as np
from scipy.spatial.transform import Rotation
import tf2_ros

CAMERA_CONFIG_PATH = "/ros2_ws/src/camera/config/camera_calibration_config.yaml"

class BEVPosePublisher(Node):
    def __init__(self):
        super().__init__('bve_pose_publisher')
        
        # Настройки QoS для разных типов данных
        self.image_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.pose_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers with QoS settings
        self.pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, 
            '/bve_pose', 
            qos_profile=self.pose_qos
        )
        
        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, 
            '/initialpose', 
            qos_profile=self.pose_qos
        )
        
        self.enemy_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, 
            '/enemy_pose', 
            qos_profile=self.pose_qos
        )

        # Subscription with QoS settings
        self.image_subscription = self.create_subscription(
            Image, 
            '/image_raw', 
            self.image_callback, 
            qos_profile=self.image_qos
        )

        self.bridge = CvBridge()
        self.camera = Camera(CAMERA_CONFIG_PATH)

        # self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, 
            self, 
            qos=self.pose_qos,
            spin_thread=True
        )
        
        # Variables to store robot and enemy pose data
        self.robotCoord = None
        self.quat = None
        self.enemyCoord = None
        self.enemyQuat = None
        self.aruco_to_base_link = self.tf_buffer.lookup_transform("aruco_link", "base_footprint", rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=2.0))


        # Sliding window for covariance estimation
        self.counter = 0  # Counter for initial pose publication

    def ensure_positive_semidefinite(self, matrix, epsilon=1e-6):
        """Ensure the covariance matrix is positive semi-definite."""
        matrix = matrix + np.eye(matrix.shape[0]) * epsilon
        eigvals, eigvecs = np.linalg.eigh(matrix)
        eigvals[eigvals < 0] = epsilon
        return eigvecs @ np.diag(eigvals) @ eigvecs.T

    def project_quaternion_to_xy_plane(self, quat):
        """Project quaternion to the xy-plane, keeping only yaw (rotation around z)."""
        # Convert quaternion to Euler angles (xyz convention)
        rotation = Rotation.from_quat(quat)
        euler = rotation.as_euler('xyz', degrees=False)
        
        # Keep only yaw (rotation around z), set roll and pitch to 0
        yaw_only_euler = [0.0, 0.0, euler[2]]
        
        # Convert back to quaternion
        projected_rotation = Rotation.from_euler(seq='xyz', angles=yaw_only_euler, degrees=False)
        return projected_rotation.as_quat()

    def image_callback(self, msg):
        """Callback function for processing incoming images."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Get pose data from Camera class
        robotCoord, quat, enemyCoord, enemyQuat, robot_cov, enemy_cov = self.camera.robots_tracking(cv_image)

        # Process our robot's pose
        if robotCoord is not None:
            self.robotCoord = robotCoord
            self.quat = self.project_quaternion_to_xy_plane(quat)  # Project quaternion to xy-plane

            # Prepare pose message
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header = Header(frame_id='map', stamp=self.get_clock().now().to_msg())
            pose_msg.pose.pose.position.x = float(self.robotCoord[0] + self.aruco_to_base_link.transform.translation.x)
            pose_msg.pose.pose.position.y = float(self.robotCoord[1] + self.aruco_to_base_link.transform.translation.y)
            pose_msg.pose.pose.position.z = float(self.robotCoord[2] + self.aruco_to_base_link.transform.translation.z)
            pose_msg.pose.pose.orientation.x = self.quat[0] 
            pose_msg.pose.pose.orientation.y = self.quat[1]
            pose_msg.pose.pose.orientation.z = self.quat[2]
            pose_msg.pose.pose.orientation.w = self.quat[3]

            covariance_matrix = np.eye(6)


            covariance_matrix[0, 0] = 0.003  # x variance
            covariance_matrix[1, 1] = 0.003  # y variance
            covariance_matrix[2, 2] = 0.003  # z variance
            covariance_matrix[3, 3] = 0.001  # roll variance
            covariance_matrix[4, 4] = 0.001  # pitch variance
            covariance_matrix[5, 5] = 0.005 # yaw variance
            pose_msg.pose.covariance = covariance_matrix.flatten().tolist()

            # Compute covariance
            # if len(self.data_window) == self.window_size:
            #     data = np.array(self.data_window).T
            #     window_cov = np.cov(data)
            #     window_cov = self.ensure_positive_semidefinite(window_cov)
            #     combined_cov = self.combine_covariances(robot_cov, window_cov, alpha=0.3)
            #     pose_msg.pose.covariance = combined_cov.flatten().tolist()
            # else:
            #     covariance_matrix = robot_cov if robot_cov is not None else np.zeros((6, 6))
            #     covariance_matrix[0, 0] = max(covariance_matrix[0, 0], 0.15)  # x variance
            #     covariance_matrix[1, 1] = max(covariance_matrix[1, 1], 0.15)  # y variance
            #     covariance_matrix[2, 2] = max(covariance_matrix[2, 2], 0.0001)  # z variance
            #     covariance_matrix[3, 3] = max(covariance_matrix[3, 3], 0.001)  # roll variance
            #     covariance_matrix[4, 4] = max(covariance_matrix[4, 4], 0.001)  # pitch variance
            #     covariance_matrix[5, 5] = max(covariance_matrix[5, 5], 0.25)  # yaw variance
            #     pose_msg.pose.covariance = covariance_matrix.flatten().tolist()

            # Publish initial pose once and regular pose
            if self.counter == 0:
                self.initial_pose_publisher.publish(pose_msg)
                self.counter = 1
                imu_pub = Vector3()
                imu_pub.x = pose_msg.pose.pose.position.x
                imu_pub.y = pose_msg.pose.pose.position.y
                imu_pub.z = Rotation.from_quat(self.quat).as_euler(seq='xyz', degrees=False)[2]
                # self.get_logger().info(f"{imu_pub}")
            self.pose_publisher.publish(pose_msg)

            # Debug output
            # self.get_logger().info(
            #     f"Our robot: x={self.robotCoord[0]:.3f}, y={self.robotCoord[1]:.3f}, z={self.robotCoord[2]:.3f}, "
            #     f"quat=[{self.quat[0]:.3f}, {self.quat[1]:.3f}, {self.quat[2]:.3f}, {self.quat[3]:.3f}]"
            # )

        # Process enemy robot's pose
        # if enemyCoord is not None:
        #     self.enemyCoord = enemyCoord
        #     self.enemyQuat = self.project_quaternion_to_xy_plane(enemyQuat)  # Project quaternion to xy-plane

            enemy_rotation = Rotation.from_quat(self.enemyQuat)
            enemy_euler = enemy_rotation.as_euler('xyz', degrees=False)
            
            # Prepare enemy pose message
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header = Header(frame_id='map', stamp=self.get_clock().now().to_msg())
            pose_msg.pose.pose.position.x = float(self.enemyCoord[0])
            pose_msg.pose.pose.position.y = float(self.enemyCoord[1])
            pose_msg.pose.pose.position.z = float(self.enemyCoord[2])
            pose_msg.pose.pose.orientation.x = self.enemyQuat[0]
            pose_msg.pose.pose.orientation.y = self.enemyQuat[1]
            pose_msg.pose.pose.orientation.z = self.enemyQuat[2]
            pose_msg.pose.pose.orientation.w = self.enemyQuat[3]

            covariance_matrix = np.eye(6)
            
            covariance_matrix[0, 0] = 0.005  # x variance
            covariance_matrix[1, 1] = 0.005  # y variance
            covariance_matrix[2, 2] = 0.003  # z variance
            covariance_matrix[3, 3] = 0.001  # roll variance
            covariance_matrix[4, 4] = 0.001  # pitch variance
            covariance_matrix[5, 5] = 0.005 # yaw variance
            pose_msg.pose.covariance = covariance_matrix.flatten().tolist()

        #     self.enemy_pose_publisher.publish(pose_msg)

            # Debug output
            # self.get_logger().info(
            #     f"Enemy robot: x={self.enemyCoord[0]:.3f}, y={self.enemyCoord[1]:.3f}, z={self.enemyCoord[2]:.3f}, "
            #     f"quat=[{self.enemyQuat[0]:.3f}, {self.enemyQuat[1]:.3f}, {self.enemyQuat[2]:.3f}, {self.enemyQuat[3]:.3f}]"
            # )

def main(args=None):
    rclpy.init(args=args)
    bve_pose_publisher = BEVPosePublisher()
    try:
        rclpy.spin(bve_pose_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        bve_pose_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()