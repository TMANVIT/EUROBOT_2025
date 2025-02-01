import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
from cvFunctions import (
    camera_initialisation, video_capture, markers_detection, imgDrawing)
import numpy as np

CAMERA_CONFIG_PATH = "ros2_ws/src/camera/config/camera_calibration_config.yaml"

class BEVPosePublisher(Node):

    def __init__(self):
        super().__init__('bev_pose_publisher')
        self.publisher_ = self.create_publisher(Pose, '/bev_pose', 10)
        self.subscription = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()

        cvf.camera_initialisation(CAMERA_CONFIG_PATH, cameraID=2, imageHight=2560, imageWidth=1440)
        self.tmatrix = None
        self.center = None

    def image_callback(self, msg):
        """
        Callback function for the /image_raw topic subscription.
        Converts the ROS Image message to an OpenCV image and processes it.
        """
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Process the image
            ids, transMatrixDict, tvecDict = self.img_processing(cv_image)

            if ids is not None:
                tmatrix_new, center_new = self.t_matrix_building(ids, tvecDict, transMatrixDict)
                if tmatrix_new is not None:
                    self.tmatrix, self.center = tmatrix_new, center_new

                robotCoords, angles = self.robots_tracking(ids, transMatrixDict, tvecDict, self.tmatrix, self.center)
                if robotCoords:
                    for coord, angle in zip(robotCoords, angles):
                        msg = Pose()
                        msg.position.x = float(coord[0])
                        msg.position.y = float(coord[1])
                        msg.position.z = float(coord[2])
                        msg.orientation.z = float(angle)
                        self.publisher_.publish(msg)
                        self.get_logger().info(f'Publishing: {msg}')
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def img_processing(self, cv_image):
        """
        Processes the OpenCV image to detect markers and compute poses.
        """
        # Replace cvf.video_capture() with the provided image
        ids, transMatrixDict, tvecDict = cvf.markers_detection(cv_image)
        cvf.imgDrawing(cv_image)
        return ids, transMatrixDict, tvecDict

    def angle_between_vectors(self, vector1, vector2):
        cos = np.dot(vector1, vector2) / (np.linalg.norm(vector1) * np.linalg.norm(vector2))
        sin = np.linalg.norm(np.cross(vector1, vector2)) / (np.linalg.norm(vector1) * np.linalg.norm(vector2))
        angle = np.angle(cos + sin * 1.j, deg=True)
        return angle

    def t_matrix_building(self, ids, tvecDict, transMatrixDict):
        corners = []
        tMatrices = []
        tmatrix = None
        center = None
        for i in ids:
            if i in range(11, 51) and i != 47:
                corners.append(tvecDict[i])
                tMatrices.append(transMatrixDict[i])

        if len(corners) == 4:
            center = sum(np.array(corners)) / 4
            corners = sorted(list(map(lambda x: x.tolist(), corners)))
            corners[:2] = [x for _, x in sorted(zip(list(map(lambda x: x[1], corners[:2])), corners[:2]))]
            corners[2:4] = [x for _, x in sorted(zip(list(map(lambda x: x[1], corners[2:4])), corners[2:4]))]

            for i in range(4):
                corners[i] = np.array(corners[i])

            xvec = (corners[2] + corners[3] - corners[1] - corners[0]) / 2
            yvec = (corners[1] + corners[3] - corners[2] - corners[0]) / 2
            xvec *= -1
            zvec = np.cross(xvec, yvec)

            tmatrix = sum(np.array(tMatrices)) / 4

        return tmatrix, center

    def robots_tracking(self, ids, transMatrixDict, tvecDict, tmatrix, center):
        robotCoords = []
        angles = []

        for i in ids:
            if i in range(1, 10):
                robotCoords.append(np.dot(np.linalg.inv(tmatrix), np.array(tvecDict[i] - center)) * 100)
                angles.append(self.angle_between_vectors(transMatrixDict[i][0], tmatrix[0]))

        return robotCoords, angles


def main(args=None):
    rclpy.init(args=args)

    bve_pose_publisher = BEVPosePublisher()

    rclpy.spin(bve_pose_publisher)

    bve_pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()