import rclpy 
from rclpy.node import Node 
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
import cv2
 
class ImageRawPublisher(Node):
  def __init__(self):
    super().__init__('image_publisher')
      
    self.publisher_ = self.create_publisher(Image, 'image_raw', 10)
      
    timer_period = 0.1  # seconds
    image_height = 2560
    image_width = 1440

    self.timer = self.create_timer(timer_period, self.timer_callback)
    self.cap = cv2.VideoCapture(0)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, image_height + 10)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, image_width + 10)

    self.br = CvBridge()
   
  def timer_callback(self):
    ret, frame = self.cap.read()
          
    if ret == True:
      self.publisher_.publish(self.br.cv2_to_imgmsg(frame))
 
    # Display the message on the console
    self.get_logger().info('Publishing video frame')
  
def main(args=None):
    rclpy.init(args=args)
  
  image_publisher = ImagePublisher()
  
  # Spin the node so the callback function is called.
  rclpy.spin(image_publisher)
  
  image_publisher.destroy_node()
  
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()