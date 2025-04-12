import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import UInt8
from scipy.spatial.transform import Rotation as R
import numpy as np

class Strategy(Node):
    def __init__(self):
        super().__init__('strategy_node')
        self.action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.elevator_pub = self.create_publisher(UInt8, '/elevator/response', 10)
        self.elevator_sub = self.create_subscription(
            UInt8,
            '/elevator/answer',
            self.elevator_callback,
            10
        )
        raw_waypoints = self.declare_parameter('waypoints')
        raw_waypoints = self.get_parameter("waypoints").get_parameter_value().double_array_value
        self.waypoints = []
        for i in range(0, len(raw_waypoints), 3): 
            self.waypoints.append({
                'x': raw_waypoints[i],
                'y': raw_waypoints[i+1],
                'yaw': raw_waypoints[i+2]
            })
        self.current_waypoint = 0
        self.navigation_in_progress = False
        self.elevator_in_progress = False
        self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        self.get_logger().debug(f"{self.waypoints}")
        if not self.navigation_in_progress and self.current_waypoint < len(self.waypoints):
            self.navigate_to_waypoint(self.waypoints[self.current_waypoint])

    def navigate_to_waypoint(self, waypoint):
        self.navigation_in_progress = True
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = waypoint['x']
        goal_msg.pose.pose.position.y = waypoint['y']
        yaw = waypoint['yaw']
        
        # Correct quaternion conversion
        quat = R.from_euler('z', yaw, degrees=False).as_quat()
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
        self.get_logger().info("Test info")
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.navigation_in_progress = False
            return

        self.get_logger().info('Goal accepted')
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.get_logger().info("Call_result_callback")
        self.navigation_in_progress = False
        self.current_waypoint += 14

    def elevator_callback(self, msg):
        self.get_logger().info(f"Received elevator response: {msg.data}")

    def feedback_callback(self, feedback_msg):
        self.get_logger().debug("Feedback_callback")

def main(args=None):
    rclpy.init(args=args)
    node = Strategy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()