import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
import cv2
import numpy as np
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import os
import yaml

class EnemyMapNode(Node):
    def __init__(self):
        super().__init__('enemy_map_node')
        
        self.map_resolution = 0.05
        self.map_width = 620
        self.map_height = 420
        self.radius_m = 0.3
        
        self.radius_px = int(self.radius_m / self.map_resolution)
        self.center_x = self.map_width // 2
        self.center_y = self.map_height // 2
        self.max_x_m = (self.map_width / 2) * self.map_resolution
        self.max_y_m = (self.map_height / 2) * self.map_resolution
        
        self.base_map = np.ones((self.map_height, self.map_width), dtype=np.uint8) * 255
        
        decks = [
            [(15, 370), (35, 290)],
            [(15, 185), (35, 105)],
            [(605, 370), (585, 290)],
            [(605, 185), (585, 105)],
            [(310-40, 210), (310-120, 210+20)],
            [(310+40, 210), (310+120, 210+20)],
            [(125, 370), (205, 350)],
            [(495, 370), (415, 350)],
            [(10, 10), (610, 410)],
            [(140, 50), (220, 10)],
            [(480, 50), (400, 10)],
            [(400, 10), (220, 100)],
            [(310-95, 55), (310-175, 75)],
            [(310+95, 55), (310+175, 75)]
        ]
        
        for deck in decks:
            cv2.rectangle(self.base_map, deck[0], deck[1], 0, -1)
        
        self.current_map = self.base_map.copy()
        
        self.pose_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/enemy_pose',
            self.enemy_pose_callback,
            qos_profile=self.pose_qos
        )
        
        map_qos = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.map_publisher = self.create_publisher(OccupancyGrid, '/dynamic_map', map_qos)
        self.timer = self.create_timer(0.1, self.publish_and_save_map)
        
        # Путь для сохранения карты
        self.map_dir = '/ros2_ws/src/navigation/map/'
        self.map_name = 'battlefield'
        self.save_map()  # Сохраняем начальную карту
        
        self.get_logger().info('Enemy Map Node has been started')

    def enemy_pose_callback(self, msg):
        self.current_map = self.base_map.copy()
        enemy_x = msg.pose.pose.position.x
        enemy_y = msg.pose.pose.position.y
        
        if (abs(enemy_x) > self.max_x_m + self.radius_m or 
            abs(enemy_y) > self.max_y_m + self.radius_m):
            self.get_logger().warn(f'Enemy position ({enemy_x:.2f}, {enemy_y:.2f}) is completely outside map boundaries')
            return
        
        pixel_x = self.center_x + int(enemy_x / self.map_resolution)
        pixel_y = self.center_y - int(enemy_y / self.map_resolution)
        
        if (pixel_x + self.radius_px >= 0 and pixel_x - self.radius_px < self.map_width and
            pixel_y + self.radius_px >= 0 and pixel_y - self.radius_px < self.map_height):
            pixel_x = max(0, min(pixel_x, self.map_width - 1))
            pixel_y = max(0, min(pixel_y, self.map_height - 1))
            cv2.circle(self.current_map, (pixel_x, pixel_y), self.radius_px, 0, -1)
        else:
            self.get_logger().info(f'Enemy circle at ({enemy_x:.2f}, {enemy_y:.2f}) is outside visible map area')

    def save_map(self):
        # Сохранение PGM-файла
        pgm_path = os.path.join(self.map_dir, f'{self.map_name}.pgm')
        cv2.imwrite(pgm_path, self.current_map)
        
        # Создание YAML-файла
        yaml_data = {
            'image': f'{self.map_name}.pgm',
            'resolution': self.map_resolution,
            'origin': [-self.center_x * self.map_resolution, -self.center_y * self.map_resolution, 0.0],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.196
        }
        yaml_path = os.path.join(self.map_dir, f'{self.map_name}.yaml')
        with open(yaml_path, 'w') as f:
            yaml.dump(yaml_data, f)
        
        self.get_logger().info(f'Map saved to {yaml_path}')

    def publish_and_save_map(self):
        map_msg = OccupancyGrid()
        map_msg.header = Header()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = "map"

        
        # Сохраняем карту в файл при каждом обновлении
        self.save_map()

def main(args=None):
    rclpy.init(args=args)
    node = EnemyMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()