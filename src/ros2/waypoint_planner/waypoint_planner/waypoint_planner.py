import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

class WayPointPlanner(Node):
    def __init__(self):
        super().__init__('waypoint_planner_node')
        self.action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.pub_signal = self.create_publisher(String, '/action_signal', 10)
        self.elevator_responce = self.create_subscription(
            String,
            '/elevator_response',
            self.handle_action_response,
            10
        )
        self.waypoints = self.declare_parameter('waypoints', []).value
        self.current_waypoint = 0

    def start_navigation(self):
        if self.current_waypoint < len(self.waypoints):
            self.navigate_to_waypoint(self.waypoints[self.current_waypoint])

    def navigate_to_waypoint(self, waypoint):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = waypoint['x']
        goal_msg.pose.pose.position.y = waypoint['y']
        goal_msg.pose.pose.orientation.w = 1.0

        self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected!')
            return
        self.get_logger().info('Goal accepted!')
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result:
            self.get_logger().info(f'Reached waypoint {self.current_waypoint + 1}')
            self.trigger_external_action()

    def trigger_external_action(self):
        signal_msg = String()
        signal_msg.data = f'action_for_waypoint_{self.current_waypoint + 1}'
        self.pub_signal.publish(signal_msg)
        self.get_logger().info(f'Sent signal: {signal_msg.data}')

    def handle_action_response(self, msg):
        if msg.data == 'action_completed':
            self.get_logger().info('External action completed!')
            self.current_waypoint += 1
            self.start_navigation()

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Distance remaining: {feedback_msg.feedback.distance_remaining:.2f}m')

def main(args=None):
    rclpy.init(args=args)
    node = WayPointPlanner()
    node.start_navigation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()