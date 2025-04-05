import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageRawPublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        
        # Оптимизированный QoS профиль для видео
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Лучше подходит для видео
            history=QoSHistoryPolicy.KEEP_LAST,           # Хранить только последние
            depth=1,                                      # Только 1 сообщение в буфере
            durability=2                                  # VOLATILE (по умолчанию)
        )
        
        self.publisher_ = self.create_publisher(Image, 'image_raw', qos_profile)
        
        # Параметры камеры
        self.camera_params = {
            'width': 2560,
            'height': 1440,
            'fps': 30,
            'format': 'MJPG'
        }
        
        # Инициализация камеры
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_params['width'])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_params['height'])
        self.cap.set(cv2.CAP_PROP_FPS, self.camera_params['fps'])
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.camera_params['format']))
        
        self.br = CvBridge()
        
        # Таймер с учетом FPS
        timer_period = 1.0 / self.camera_params['fps']  # секунды
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def timer_callback(self):
        ret, frame = self.cap.read()
        
        if ret:
            # try:
                # Публикация сжатого изображения для экономии полосы
          msg = self.br.cv2_to_imgmsg(frame, encoding="bgr8")
          msg.header.stamp = self.get_clock().now().to_msg()
          self.publisher_.publish(msg)
            # except Exception as e:
            #     self.get_logger().error(f'Error publishing image: {str(e)}')
    
    def __del__(self):
        self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    
    image_publisher = ImageRawPublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()