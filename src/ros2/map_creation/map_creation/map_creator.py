import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
import cv2
import numpy as np
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_msgs.msg import String

class EnemyMapNode(Node):
    def __init__(self):
        super().__init__('map_creator_node')

        # Настройка QoS для публикации карты
        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        # Публикация карты
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', qos_profile)

        # Подписка на позицию противника
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/enemy_pose',
            self.enemy_pose_callback,
            rclpy.qos.QoSProfile(depth=10)
        )

        # Подписка на команды управления препятствиями
        self.obstacle_control_sub = self.create_subscription(
            String,
            '/obstacle_control',
            self.obstacle_control_callback,
            rclpy.qos.QoSProfile(depth=10)
        )

        # Параметры карты
        self.map_resolution = 0.005  # 0.5 см/пиксель
        self.map_width = 620         # 3.1 м
        self.map_height = 420        # 2.1 м
        self.origin_x = -1.55        # Центр в (0, 0)
        self.origin_y = -1.05
        self.radius_px = 60          # 0.3 м = 60 пикселей

        # Список игнорируемых препятствий
        self.ignored_obstacles = set()

        # Хранение последней позиции противника
        self.last_enemy_pose = None

        # Определение препятствий с именами
        self.decks = {
            "left_upper_material": [(15, 370), (35, 290)],
            "left_lower_material": [(15, 185), (35, 105)],
            "right_upper_material": [(605, 370), (585, 290)],
            "right_lower_material": [(605, 185), (585, 105)],
            "center_material_left": [(310-40, 210), (310-120, 210+20)],
            "center_material_right": [(310+40, 210), (310+120, 210+20)],
            "lower_material_left": [(125, 370), (205, 350)],
            "lower_material_right": [(495, 370), (415, 350)],
            "ramp_left": [(140, 50), (220, 10)],
            "ramp_right": [(480, 50), (400, 10)],
            "stage": [(400, 10), (220, 100)],
            "stage_material_left": [(310-95, 55), (310-175, 75)],
            "stage_material_right": [(310+95, 55), (310+175, 75)]
        }

        # Базовая карта: занята (0)
        self.base_map = np.ones((self.map_height, self.map_width), dtype=np.uint8)

        # Рисуем препятствия на базовой карте
        for deck in self.decks.values():
            cv2.rectangle(self.base_map, deck[0], deck[1], 100, -1)

        # Поворачиваем базовую карту
        self.base_map = cv2.rotate(self.base_map, cv2.ROTATE_180)

        # Инициализируем текущую карту как копию базовой
        self.current_map = self.base_map.copy()

        # Таймер для публикации карты с частотой 30 Гц
        self.timer = self.create_timer(0.033, self.timer_callback)

        # Первоначальная публикация карты
        self.publish_map()

    def obstacle_control_callback(self, msg):
        """Обработка команд управления препятствиями."""
        try:
            action, obstacle_name = msg.data.split(':')
            if obstacle_name not in self.decks:
                self.get_logger().warn(f'Unknown obstacle: {obstacle_name}')
                return
            if action == 'ignore':
                self.ignored_obstacles.add(obstacle_name)
                self.get_logger().info(f'Ignoring obstacle: {obstacle_name}')
            elif action == 'enable':
                self.ignored_obstacles.discard(obstacle_name)
                self.get_logger().info(f'Enabling obstacle: {obstacle_name}')
            else:
                self.get_logger().warn(f'Unknown action: {action}')
                return
            self.update_map()
        except ValueError:
            self.get_logger().warn(f'Invalid message format: {msg.data}')

    def enemy_pose_callback(self, msg):
        """Обработка позиции противника."""
        self.last_enemy_pose = msg.pose.pose
        self.update_map()

    def update_map(self):
        """Обновление карты с учетом игнорируемых препятствий и позиции противника."""
        # Копируем базовую карту
        self.current_map = self.base_map.copy()

        # Очищаем игнорируемые препятствия
        for name, deck in self.decks.items():
            if name in self.ignored_obstacles:
                cv2.rectangle(self.current_map, deck[0], deck[1], 0, -1)

        # Рисуем противника, если известна его позиция
        if self.last_enemy_pose:
            enemy_x = self.last_enemy_pose.position.x
            enemy_y = self.last_enemy_pose.position.y
            pixel_x = int((enemy_x - self.origin_x) / self.map_resolution)
            pixel_y = int((enemy_y - self.origin_y) / self.map_resolution)
            if (0 <= pixel_x < self.map_width) and (0 <= pixel_y < self.map_height):
                cv2.circle(self.current_map, (pixel_x, pixel_y), self.radius_px, 100, -1)

    def timer_callback(self):
        """Публикация актуального состояния карты."""
        self.publish_map()

    def publish_map(self):
        """Формирование и публикация сообщения карты."""
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