import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cvFunctions.cvf import Camera
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header


CAMERA_CONFIG_PATH = "/ros2_ws/src/camera/config/camera_calibration_config.yaml"

class BEVPosePublisher(Node):

    def __init__(self):
        super().__init__('bev_pose_publisher')
        self.pose_publisher = self.create_publisher(PoseStamped, '/bev_pose', 10)
        self.initial_pose_publisher = self.create_publisher(PoseStamped, '/initialpose', 10)
        self.counter = 0
        
        self.image_subscription = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.timer = self.create_timer(0.1, self.broadcast_transforms)

        self.bridge = CvBridge()
        self.camera = Camera(CAMERA_CONFIG_PATH)
        
        self.tmatrix = None
        self.center = None
        
        self.robotCoord = None
        self.quat = None

        self.initial_pose_msg = None

    def image_callback(self, msg):
        """
        Callback function for the /image_raw topic subscription.
        Converts the ROS Image message to an OpenCV image and processes it.
        """
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Process the image
        ids, transMatrixDict, tvecDict = self.camera.detect_markers(cv_image)

        if ids is not None:
            tmatrix_new, center_new = self.camera.t_matrix_building(ids, tvecDict, transMatrixDict)
            if tmatrix_new is not None:
                self.tmatrix, self.center = tmatrix_new, center_new
            #self.get_logger().error(f"{ids}")
            if self.tmatrix is not None:
                self.robotCoord, self.quat = self.camera.robots_tracking(ids, transMatrixDict, tvecDict, self.tmatrix, self.center)
            if self.robotCoord is not None:
                msg = PoseStamped()
                msg.header = Header(frame_id='aruco', stamp=self.get_clock().now().to_msg())
                msg.pose.position.x = float(self.robotCoord[0])
                msg.pose.position.y = float(self.robotCoord[1])
                msg.pose.position.z = float(self.robotCoord[2])
                msg.pose.orientation.x = self.quat[0]
                msg.pose.orientation.y = self.quat[1]
                msg.pose.orientation.z = self.quat[2]
                msg.pose.orientation.w = self.quat[3]
                if self.counter == 0:
                    self.initial_pose_msg = msg
                    self.initial_pose_publisher.publish(msg)
                    self.counter = 1
                self.pose_publisher.publish(msg)

    def broadcast_transforms(self):
        if self.initial_pose_msg is not None:
            self.initial_pose_msg.header.stamp = self.get_clock().now().to_msg()
            self.initial_pose_publisher.publish(self.initial_pose_msg)
            # self.get_logger().info(
            #     f"Initial pose updated"
            # )

def main(args=None):
    rclpy.init(args=args)

    bve_pose_publisher = BEVPosePublisher()

    rclpy.spin(bve_pose_publisher)

    bve_pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()