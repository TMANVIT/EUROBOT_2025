import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
import cv2
import numpy as np

class EnemyMapNode(Node):
    def __init__(self):
        super().__init__('map_creator_node')
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', 10)
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped, '/enemy_pose', self.enemy_pose_callback,
            rclpy.qos.QoSProfile(depth=10)
        )
        self.map_resolution = 0.005  # 0.5 см/пиксель
        self.map_width = 620         # 3.1 м
        self.map_height = 420        # 2.1 м
        self.origin_x = -1.55        # Центр в (0, 0)
        self.origin_y = -1.05
        self.radius_px = 60          # 0.3 м = 60 пикселей

        # Базовая карта: занята (0)
        self.base_map = np.ones((self.map_height, self.map_width), dtype=np.uint8)

        # Прямоугольники (в пикселях)
        self.decks = [
            [(15, 370), (35, 290)],              # left_upper_material
            [(15, 185), (35, 105)],              # left_lower_material
            [(605, 370), (585, 290)],            # right_upper_material
            [(605, 185), (585, 105)],            # right_lower_material
            [(310-40, 210), (310-120, 210+20)],  # center_material
            [(310+40, 210), (310+120, 210+20)],  # center_material
            [(125, 370), (205, 350)],            # lower_material
            [(495, 370), (415, 350)],            # lower_material
            [(10, 10), (610, 410)],              # borders
            [(140, 50), (220, 10)],              # ramp
            [(480, 50), (400, 10)],              # ramp
            [(400, 10), (220, 100)],             # stage
            [(310-95, 55), (310-175, 75)],       # stage_material
            [(310+95, 55), (310+175, 75)]        # stage_material
        ]

        # Рисуем прямоугольники значением 50 (частично занято)
        for deck in self.decks:
            cv2.rectangle(self.base_map, deck[0], deck[1], 100, 1)
            self.get_logger().info(f'Drawing rectangle: {deck[0]} to {deck[1]}')

        self.base_map = cv2.rotate(self.base_map, cv2.ROTATE_180)

        self.current_map = self.base_map.copy()
        self.get_logger().info('Enemy Map Node has been started')

    def enemy_pose_callback(self, msg):
        self.current_map = self.base_map.copy()  # Копируем карту с прямоугольниками
        enemy_x = msg.pose.pose.position.x
        enemy_y = msg.pose.pose.position.y
        pixel_x = int((enemy_x - self.origin_x) / self.map_resolution)
        pixel_y = int((enemy_y - self.origin_y) / self.map_resolution)

        if (0 <= pixel_x < self.map_width) and (0 <= pixel_y < self.map_height):
            cv2.circle(self.current_map, (pixel_x, pixel_y), self.radius_px, 100, -1)  # Свободно (100)
            self.publish_map()

    def publish_map(self):
        map_msg = OccupancyGrid()
        map_msg.header.frame_id = 'map'
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.info.resolution = self.map_resolution
        map_msg.info.width = self.map_width
        map_msg.info.height = self.map_height
        map_msg.info.origin.position.x = self.origin_x
        map_msg.info.origin.position.y = self.origin_y
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.w = 1.0
        map_msg.data = np.array(self.current_map.flatten(), dtype=np.int8).tolist()
        self.map_publisher.publish(map_msg)

def main(args=None):
    rclpy.init(args=args)
    node = EnemyMapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()