import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cvFunctions.cvf import Camera
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Header
import tf2_ros
from numpy import zeros


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

    def image_callback(self, msg):
        """
        Callback function for the /image_raw topic subscription.
        Converts the ROS Image message to an OpenCV image and processes it.
        """
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Process the image
        ids, transMatrixDict, tvecDict = self.camera.detect_markers(cv_image)

        if ids is not None:
            tmatrix_new, center_new = self.camera.t_matrix_building(ids, tvecDict)
            if tmatrix_new is not None:
                if self.tmatrix is None:
                    self.tmatrix, self.center = tmatrix_new, center_new
                else:
                    self.tmatrix, self.center = 0.1*(tmatrix_new-self.tmatrix) + self.tmatrix, 0.1*(center_new-self.center) + self.center
            #self.get_logger().error(f"{ids}")
            if self.tmatrix is not None:
                self.robotCoord, self.quat, self.ourRobot = self.camera.robots_tracking(ids, transMatrixDict, tvecDict, self.tmatrix, self.center)
            if self.robotCoord is not None:
                msg = PoseWithCovarianceStamped()
                msg.header = Header(frame_id='map', stamp=self.get_clock().now().to_msg())
                msg.pose.pose.position.x = float(self.robotCoord[0])
                msg.pose.pose.position.y = float(self.robotCoord[1])
                msg.pose.pose.position.z = float(self.robotCoord[2])
                msg.pose.pose.orientation.x = self.quat[0]
                msg.pose.pose.orientation.y = self.quat[1]
                msg.pose.pose.orientation.z = self.quat[2]
                msg.pose.pose.orientation.w = self.quat[3] * -1

                covariance_matrix = zeros((6, 6))
                covariance_matrix[0, 0] = 0.07  # sigma_x^2
                covariance_matrix[1, 1] = 0.07  # sigma_y^2
                covariance_matrix[2, 2] = 0.0001  # sigma_z^2 
                covariance_matrix[3, 3] = 0.001  # sigma_roll^2
                covariance_matrix[4, 4] = 0.001  # sigma_pitch^2
                covariance_matrix[5, 5] = 0.15  # sigma_yaw^2 

                msg.pose.covariance = covariance_matrix.flatten().tolist()


                if self.ourRobot:
                    if self.counter == 0:
                        self.initial_pose_publisher.publish(msg)
                        self.counter = 1
                    self.pose_publisher.publish(msg)
                else:
                    self.enemy_pose_publisher.publish(msg)

    

def main(args=None):
    rclpy.init(args=args)

    bve_pose_publisher = BEVPosePublisher()

    rclpy.spin(bve_pose_publisher)

    bve_pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()