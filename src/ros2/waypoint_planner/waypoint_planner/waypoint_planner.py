import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import UInt8
from scipy.spatial.transform import Rotation as R
import numpy as np

class WayPointPlanner(Node):
    def __init__(self):
        super().__init__('waypoint_planner_node')
        self.action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.elevator_pub = self.create_publisher(UInt8, '/elevator/response', 10)
        self.elevator_responce = self.create_subscription(
            UInt8,
            '/elevator/answer',
            self.elevator_callback,
            10
        )
        self.waypoints = self.declare_parameter('waypoints', []).value
        self.create_timer(0.1, self.start_navigation())
        self.current_waypoint = 0
        self.counter = False

    def start_navigation(self):
        if not self.counter:
            self.navigate_to_waypoint(self.waypoints[self.current_waypoint])

    def navigate_to_waypoint(self, waypoint):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = waypoint['x']
        goal_msg.pose.pose.position.y = waypoint['y']
        yaw = waypoint['yaw']
        
        quat = R.from_euler(seq='xyz', angles=np.array([0, 0, yaw]), degrees=False)
        goal_msg.pose.pose.orientation.x = quat[0] 
        goal_msg.pose.pose.orientation.y = quat[1]
        goal_msg.pose.pose.orientation.z = quat[2]
        goal_msg.pose.pose.orientation.w = quat[3]

        self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.current_waypoint += 1
        self.counter = True
        
    def elevator_callback(self, msg):
        pass

    ## Need to debug
    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Current waypoint: {self.current_waypoint}')

def main(args=None):
    rclpy.init(args=args)
    node = WayPointPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()