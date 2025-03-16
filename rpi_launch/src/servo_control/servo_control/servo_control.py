import rclpy
from rclpy.node import Node
from .ArduinoConnect import ArduinoConnection
import numpy as np
from geometry_msgs.msg import Vector3
import struct

class ServoControl(Node):

    def __init__(self):
        super().__init__('ServoControlNode')
        
        self.declare_parameter('port', "/dev/ttyUSB0")
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('servo_speed', 4500)
        self.declare_parameter('servo_accel', 50)
        
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.servo_speed = self.get_parameter('servo_speed').get_parameter_value().integer_value
        self.servo_accel = self.get_parameter('servo_accel').get_parameter_value().integer_value
        
        self.command_subscriber = self.create_subscription(Vector3, '/waveshare/servo_topic', self.command_callback, 10)
        self.servo = ArduinoConnection(self.port, self.baudrate)
                
    def command_callback(self, msg):
        id = np.uint8(msg.x)
        angle = np.int32(msg.y)
        speed = np.uint16(self.servo_speed)
        accel = np.uint8(self.servo_accel)
        
        message = struct.pack("<BhHB", id, angle, speed, accel)
        
        self.servo.write(message)
        
def main(args=None):
    rclpy.init(args=args)

    servo_control = ServoControl()

    rclpy.spin(servo_control)

    servo_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()