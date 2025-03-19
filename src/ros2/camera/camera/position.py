import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cvFunctions.cvf import Camera
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Header
import tf2_ros
from numpy import zeros
import numpy as np
from collections import deque
from scipy.spatial.transform import Rotation

CAMERA_CONFIG_PATH = "/ros2_ws/src/camera/config/camera_calibration_config.yaml"

class BEVPosePublisher(Node):

    def __init__(self):
        super().__init__('bve_pose_publisher')
        self.pose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/bve_pose', 10)
        self.initial_pose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.enemy_pose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/enemy_pose', 10)
        self.counter = 0
        
        self.image_subscription = self.create_subscription(Image, '/image_raw', self.image_callback, 10)

        self.bridge = CvBridge()
        self.camera = Camera(CAMERA_CONFIG_PATH)
        
        self.tmatrix = None
        self.center = None
        
        self.robotCoord = None
        self.quat = None

        self.ourRobot = None

        # Скользящее окно для вычисления ковариации (60 кадров ~ 2 сек при 30 Гц)
        self.window_size = 15
        self.data_window = deque(maxlen=self.window_size)  # Для всех данных [x, y, z, roll, pitch, yaw]

    def ensure_positive_semidefinite(self, matrix, epsilon=1e-6):
        """
        Принудительно делает матрицу положительно полуопределённой, добавляя небольшую величину к диагонали.
        """
        matrix = matrix + np.eye(matrix.shape[0]) * epsilon
        eigvals, eigvecs = np.linalg.eigh(matrix)
        eigvals[eigvals < 0] = epsilon  # Заменяем отрицательные значения на малую положительную величину
        return eigvecs @ np.diag(eigvals) @ eigvecs.T

    def image_callback(self, msg):
        """
        Callback function for the /image_raw topic subscription.
        Converts the ROS Image message to an OpenCV image and processes it.
        """
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Process the image
        ids, transMatrixDict, tvecDict, weightsDict = self.camera.detect_markers(cv_image)

        if ids is not None:
            tmatrix_new, center_new = self.camera.t_matrix_building(ids, tvecDict)
            if tmatrix_new is not None:
                if self.tmatrix is None:
                    self.tmatrix, self.center = tmatrix_new, center_new
                else:
                    self.tmatrix, self.center = 0.1*(tmatrix_new-self.tmatrix) + self.tmatrix, 0.1*(center_new-self.center) + self.center
            if self.tmatrix is not None:
                self.robotCoord, self.quat, self.ourRobot = self.camera.robots_tracking(ids, transMatrixDict, tvecDict, weightsDict, self.tmatrix, self.center)
            if self.robotCoord is not None:
                # Преобразуем кватернионы в углы Эйлера (roll, pitch, yaw) с помощью scipy
                rotation = Rotation.from_quat(self.quat)  # self.quat в формате [x, y, z, w]
                euler = rotation.as_euler('xyz', degrees=False)  # 'xyz' = roll, pitch, yaw в радианах

                # Добавляем все данные в скользящее окно как один список [x, y, z, roll, pitch, yaw]
                self.data_window.append([self.robotCoord[0], self.robotCoord[1], self.robotCoord[2], 
                                        euler[0], euler[1], euler[2]])

                # Создаём сообщение
                pose_msg = PoseWithCovarianceStamped()
                pose_msg.header = Header(frame_id='map', stamp=self.get_clock().now().to_msg())
                pose_msg.pose.pose.position.x = float(self.robotCoord[0])
                pose_msg.pose.pose.position.y = float(self.robotCoord[1])
                pose_msg.pose.pose.position.z = float(self.robotCoord[2])
                pose_msg.pose.pose.orientation.x = self.quat[0]
                pose_msg.pose.pose.orientation.y = self.quat[1]
                pose_msg.pose.pose.orientation.z = self.quat[2]
                pose_msg.pose.pose.orientation.w = self.quat[3]

            
                if len(self.data_window) == self.window_size:
                    # Преобразуем данные в массив NumPy, транспонируем для np.cov
                    data = np.array(self.data_window).T  # строки = [x, y, z, roll, pitch, yaw], столбцы = измерения

                    # Вычисляем полную ковариационную матрицу 6x6
                    covariance_matrix = np.cov(data)

                    # Делаем матрицу положительно полуопределённой
                    covariance_matrix = self.ensure_positive_semidefinite(covariance_matrix)

                    # Присваиваем ковариацию сообщению
                    pose_msg.pose.covariance = covariance_matrix.flatten().tolist()
                else:
                    # Если окно ещё не заполнено, используем фиксированные значения
                    covariance_matrix = zeros((6, 6))
                    covariance_matrix[0, 0] = 0.15    # sigma_x^2
                    covariance_matrix[1, 1] = 0.15    # sigma_y^2
                    covariance_matrix[2, 2] = 0.0001  # sigma_z^2 
                    covariance_matrix[3, 3] = 0.001   # sigma_roll^2
                    covariance_matrix[4, 4] = 0.001   # sigma_pitch^2
                    covariance_matrix[5, 5] = 0.25    # sigma_yaw^2 
                    pose_msg.pose.covariance = covariance_matrix.flatten().tolist()

                # Публикуем сообщение
                if self.ourRobot:
                    if self.counter == 0:
                        self.initial_pose_publisher.publish(pose_msg)
                        self.counter = 1
                    self.pose_publisher.publish(pose_msg)
                else:
                    self.enemy_pose_publisher.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    bve_pose_publisher = BEVPosePublisher()
    rclpy.spin(bve_pose_publisher)
    bve_pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()