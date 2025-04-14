import rclpy
import time
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import UInt8, Int16
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import String
class Strategy(Node):
    def __init__(self):
        super().__init__('strategy_node')
        # Create Action
        self.action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        
        # Init Pub/Sub
        self.elevator_pub = self.create_publisher(UInt8, '/elevator/response', 10)
        self.elevator_sub = self.create_subscription(UInt8,'/elevator/answer', self.elevator_callback, 10)
        
        self.timer_sub = self.create_subscription(UInt8, '/timer', self.timer_counter, 10)
        self.obstacle_pub = self.create_publisher(String, '/obstacle_control', 10)
        
        self.screen_pub = self.create_publisher(Int16, '/screen', 10)
        
        self.create_timer(0.1, self.timer_callback)
        
        # Declare parameters
        raw_waypoints = self.declare_parameter('waypoints')
        self.elevator_order = self.declare_parameter('elevator_order')
        self.time_until_end = self.declare_parameter('time_until_end')
        self.obstacles = self.declare_parameter('obstacle_release')
        
        # Read params from YAML
        raw_waypoints = self.get_parameter("waypoints").get_parameter_value().double_array_value
        self.waypoints = []
        for i in range(0, len(raw_waypoints), 3): 
            self.waypoints.append({
                'x': raw_waypoints[i],
                'y': raw_waypoints[i+1],
                'yaw': raw_waypoints[i+2]
            })
        self.elevator_order = self.get_parameter("elevator_order").get_parameter_value().integer_array_value
        self.time_until_end = self.get_parameter("time_until_end").get_parameter_value().integer_value
        self.obstacles = self.get_parameter("obstacles").get_parameter_value().string_array_value    
        
        # Node Variables
        self.action_tact = 0 # Number set of actions
        self.navigation_in_progress = False # True means that robot is going to point
        self.elevator_in_progress = False # True means that robot work with elevator
        self.start_timer = None # Timing variable. Need to synchronize with SIMa's
        self.current_elevator = 0 # Index of current elevator command
        self.current_waypoint = 0 # Index of current goal point
        self.current_obstacle = 0 # Index of current obstacle
        
    def timer_callback(self):
        # Start algo only when initialize timer
        if self.start_timer is not None:
            # Check if match near the end
            if time.time() - self.start_timer >= self.time_until_end:
                self.navigate_to_waypoint(self.navigate_to_waypoint[len(self.waypoints)])
            
            # First tact - publish paint on the side of the field.
            if self.action_tact == 0:
                if not self.elevator_in_progress:
                    self.elevator_publish(self.current_elevator)
            
            # Second tact - go to first cans, grab them, release them from map.         
            if self.action_tact == 1 and time.time() - self.start_timer < 20: ###### TODO Tune time of interrupt Tact.
                if not self.navigation_in_progress:
                    self.navigate_to_waypoint(self.current_waypoint)
                if not self.navigation_in_progress and self.current_waypoint == 1:
                    if not self.elevator_in_progress:
                        self.elevator_publish(self.current_elevator)                        
            elif self.action_tact == 1:
                self.action_tact += 1
            
            # Third tact - go to first base, build on them tribunes.    
            if self.action_tact == 2 and time.time() - self.start_timer < 40: ###### TODO Tune time of interrupt Tact.
                if not self.navigation_in_progress:
                    self.navigate_to_waypoint(self.current_waypoint)
                if not self.navigation_in_progress and self.current_waypoint == 2:
                    if not self.elevator_in_progress:
                        self.elevator_publish(self.current_elevator) 
            elif self.action_tact == 2:
                self.action_tact += 1
            
            # Fourth tact - go to second cans, grab them, release them from map.
            if self.action_tact == 3 and time.time() - self.start_timer < 60: ###### TODO Tune time of interrupt Tact.
                if not self.navigation_in_progress:
                    self.navigate_to_waypoint(self.current_waypoint)
                if not self.navigation_in_progress and self.current_waypoint == 3:
                    if not self.elevator_in_progress:
                        self.elevator_publish(self.current_elevator)
            elif self.action_tact == 3:
                self.action_tact += 1
            
            # Fifth tact - go to second base, build on them tribunes.     
            if self.action_tact == 4 and time.time() - self.start_timer < 80: ###### TODO Tune time of interrupt Tact.
                if not self.navigation_in_progress:
                    self.navigate_to_waypoint(self.current_waypoint)
                if not self.navigation_in_progress and self.current_waypoint == 4:
                    if not self.elevator_in_progress:
                        self.elevator_publish(self.current_elevator) 
            elif self.action_tact == 4:
                self.action_tact += 1
            
            # Sixth tact - go to the 'waiting' point until SIMA's start time.
            if self.action_tact == 5 and time.time() - self.start_timer < 80: ###### TODO Tune time of interrupt Tact.
                if not self.navigation_in_progress:
                    self.navigate_to_waypoint(self.current_waypoint)
                if not self.navigation_in_progress and self.current_waypoint == 5:
                    if not self.elevator_in_progress:
                        self.elevator_publish(self.current_elevator)
            elif self.action_tact == 5:
                self.action_tact += 1

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
    
    def feedback_callback(self, feedback_msg):
        self.get_logger().debug("Feedback_callback")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Tact failed')
            self.action_tact += 1
            return
        
        self.get_logger().debug('Goal accepted')
        self.navigation_in_progress = False
        self.current_waypoint += 1

    def elevator_publish(self, command_index):
        self.elevator_in_progress = True
        msg = UInt8()
        msg.data = command_index
        self.elevator_pub.publish(msg.data)
    
    def elevator_callback(self, msg):
        if msg.data == self.current_elevator:
            if self.action_tact == 0:
                self.elevator_in_progress = False
                self.current_elevator += 1
                self.action_tact += 1
            if self.action_tact == 1:
                self.elevator_in_progress = False
                self.current_elevator += 1
                self.action_tact += 1
                self.update_obstacle('ignore', self.obstacles[self.current_obstacle])
            if self.action_tact == 2:
                self.elevator_in_progress = False
                self.current_elevator += 1
                self.action_tact += 1
            if self.action_tact == 3:
                self.elevator_in_progress = False
                self.current_elevator += 1
                self.action_tact += 1
                self.update_obstacle('ignore', self.obstacles[self.current_obstacle])
            if self.action_tact == 4:
                self.elevator_in_progress = False
                self.current_elevator += 1
                self.action_tact += 1
            if self.action_tact == 5:
                self.elevator_in_progress = False
                self.current_elevator += 1
                self.action_tact += 1
        
    def update_obstacle(self, action, obstacle_name):
        msg = String()
        msg.data = f'{action}:{obstacle_name}'
        self.obstacle_pub.publish(msg)
        self.get_logger().debug(f'Sent command: {msg.data}')
        self.current_obstacle += 1
    
    # Main synchronize function
    def timer_counter(self, msg):
        if self.start_timer is None and msg.data == 1:
            self.start_timer = time.time()

def main(args=None):
    rclpy.init(args=args)
    node = Strategy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()