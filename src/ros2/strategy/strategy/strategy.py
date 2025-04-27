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
        self.elevator_pub = self.create_publisher(UInt8, '/elevator/request', 10)
        self.elevator_sub = self.create_subscription(UInt8,'/elevator/answer', self.elevator_callback, 10)
        
        self.timer_sub = self.create_subscription(UInt8, '/timer', self.timer_counter, 10)
        self.obstacle_pub = self.create_publisher(String, '/obstacle_control', 10)
        
        self.screen_pub = self.create_publisher(Int16, '/screen', 10)
        
        self.create_timer(1, self.timer_callback)
        self.create_timer(3, self.screen_callback)
        
        # Declare parameters
        raw_waypoints = self.declare_parameter('waypoints')
        self.elevator_order = self.declare_parameter('elevator_order')
        self.time_until_end = self.declare_parameter('time_until_end')
        self.obstacles = self.declare_parameter('obstacle_release')
        self.term_order = self.declare_parameter('term_order')
        
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
        self.obstacles = self.get_parameter("obstacle_release").get_parameter_value().string_array_value  
        self.term_order = self.get_parameter('term_order').get_parameter_value().integer_array_value 
        
        # Node Variables
        self.action_tact = 0 # Number set of actions
        self.navigation_in_progress = False # True means that robot is going to point
        self.elevator_in_progress = False # True means that robot work with elevator
        self.start_timer = None # Timing variable. Need to synchronize with SIMa's
        self.current_elevator = 0 # Index of current elevator command
        self.current_waypoint = 0 # Index of current goal point
        self.current_obstacle = 0 # Index of current obstacle
        self.current_term = 0 # Index of current term
        self.screen_sum = 40 # Summary value of points. Include SIMAs
        
    def timer_callback(self):
        # Start algo only when initialize timer
        if self.start_timer is not None:
            self.get_logger().info(f"Time = {time.time() - self.start_timer}")
            # Check if match near the end
            if (time.time() - self.start_timer) >= self.time_until_end and (time.time() - self.start_timer) < 100:
                self.get_logger().info(f"Len wayp = {len(self.waypoints)}")
                if len(self.waypoints) == self.current_waypoint + 1:
                    self.navigate_to_waypoint(len(self.waypoints) - 1)
                    
            # First tact - publish paint on the side of the field.
            elif self.action_tact == 0:
                if not self.elevator_in_progress:
                    self.update_obstacle('ignore', self.obstacles[self.current_obstacle])
                    self.elevator_publish(self.current_elevator)
            
            # Second tact - go to first cans, grab them, release them from map.         
            elif self.action_tact == 1 and time.time() - self.start_timer < 20: ###### TODO Tune time of interrupt Tact.
                if not self.navigation_in_progress and self.current_waypoint == 0:
                    self.navigate_to_waypoint(self.current_waypoint)
                if not self.navigation_in_progress and self.current_waypoint == 1:
                    if not self.elevator_in_progress:
                        self.elevator_publish(self.current_elevator)                        
            
            # Third tact - go to first base, build on them tribunes.    
            elif self.action_tact == 2 and time.time() - self.start_timer < 40: ###### TODO Tune time of interrupt Tact.
                if not self.navigation_in_progress and self.current_waypoint == 1:
                    self.navigate_to_waypoint(self.current_waypoint)
                if not self.navigation_in_progress and self.current_waypoint == 2:
                    if not self.elevator_in_progress:
                        self.elevator_publish(self.current_elevator) 
            
            # Fourth tact - go to second cans, grab them, release them from map.
            elif self.action_tact == 3 and time.time() - self.start_timer < 60: ###### TODO Tune time of interrupt Tact.
                if not self.navigation_in_progress and self.current_waypoint == 2:
                    self.navigate_to_waypoint(self.current_waypoint)
                if not self.navigation_in_progress and self.current_waypoint == 3:
                    if not self.elevator_in_progress:
                        self.elevator_publish(self.current_elevator)
                        self.update_obstacle('ignore', self.obstacles[self.current_obstacle])
            
            # Fifth tact - go to second base, build on them tribunes.     
            elif self.action_tact == 4 and time.time() - self.start_timer < 80: ###### TODO Tune time of interrupt Tact.
                if not self.navigation_in_progress and self.current_waypoint == 3:
                    self.navigate_to_waypoint(self.current_waypoint)
                if not self.navigation_in_progress and self.current_waypoint == 4:
                    if not self.elevator_in_progress:
                        self.elevator_publish(self.current_elevator) 
            
            # Sixth tact - go to the 'waiting' point until SIMA's start time.
            elif self.action_tact == 5 and time.time() - self.start_timer < 90: ###### TODO Tune time of interrupt Tact.
                if not self.navigation_in_progress and self.current_waypoint == 4:
                    self.navigate_to_waypoint(self.current_waypoint)
                if not self.navigation_in_progress and self.current_waypoint == 5:
                    if not self.elevator_in_progress:
                        self.elevator_publish(self.current_elevator)
                        
            if (time.time() - self.start_timer > 100) or self.current_waypoint > len(self.waypoints):
                pass
            

    def navigate_to_waypoint(self, waypoint_index):
        self.navigation_in_progress = True
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        self.get_logger().info(f"Waypoint index = {waypoint_index}")
        goal_msg.pose.pose.position.x = self.waypoints[waypoint_index]['x']
        goal_msg.pose.pose.position.y = self.waypoints[waypoint_index]['y']
        yaw = self.waypoints[waypoint_index]['yaw']
        
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
            # self.action_tact += 1
            self.navigation_in_progress = False
            return
        
        self.get_logger().debug('Goal accepted')
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        self.get_logger().info("Call_result_callback")
        self.navigation_in_progress = False
        self.current_waypoint += 1
        if len(self.waypoints) == self.current_waypoint:
            self.screen_sum += self.term_order[len(self.term_order) - 1]

    def elevator_publish(self, command_index):
        self.elevator_in_progress = True
        msg = UInt8()
        msg.data = self.elevator_order[command_index]
        self.get_logger().debug(f"Publish to elevator {msg.data}")
        self.elevator_pub.publish(msg)
    
    def elevator_callback(self, msg):
        self.get_logger().info(f"Elevator callback {msg.data}")
        if msg.data == self.elevator_order[self.current_elevator]:
            if self.action_tact == 0:
                self.elevator_in_progress = False
                self.current_elevator += 1
                self.screen_sum += self.term_order[self.current_term]
                self.current_term += 1
                self.action_tact += 1
                self.get_logger().debug(f"Tact index = {self.action_tact}")
            elif self.action_tact == 1:
                self.elevator_in_progress = False
                self.current_elevator += 1
                self.action_tact += 1
                self.get_logger().debug(f"Tact index = {self.action_tact}")
            elif self.action_tact == 2:
                self.elevator_in_progress = False
                self.current_elevator += 1
                self.screen_sum += self.term_order[self.current_term]
                self.current_term += 1
                self.action_tact += 1
                self.get_logger().debug(f"Tact index = {self.action_tact}")
            elif self.action_tact == 3:
                self.elevator_in_progress = False
                self.current_elevator += 1
                self.action_tact += 1
                self.get_logger().debug(f"Tact index = {self.action_tact}")
                # self.update_obstacle('ignore', self.obstacles[self.current_obstacle])
            elif self.action_tact == 4:
                self.elevator_in_progress = False
                self.current_elevator += 1
                self.screen_sum += self.term_order[self.current_term]
                self.current_term += 1
                self.action_tact += 1
                self.get_logger().debug(f"Tact index = {self.action_tact}")
            elif self.action_tact == 5:
                self.elevator_in_progress = False
                self.current_elevator += 1
                self.action_tact += 1
                self.get_logger().debug(f"Tact index = {self.action_tact}")
        
    def update_obstacle(self, action, obstacle_name):
        msg = String()
        msg.data = f'{action}:{obstacle_name}'
        self.obstacle_pub.publish(msg)
        #self.get_logger().debug(f'Sent command: {msg.data}')
        self.current_obstacle += 1
    
    # Main synchronize function
    def timer_counter(self, msg):
        if self.start_timer is None and msg.data == 1:
            self.get_logger().debug("Start Match!")
            self.start_timer = time.time()

    # Points publisher to screen
    def screen_callback(self):
        msg = Int16()
        msg.data = self.screen_sum
        self.get_logger().debug(f"Current sum of points = {self.screen_sum}")
        self.screen_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Strategy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()