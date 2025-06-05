import time

from arduino_connect import ArduinoConnection
import struct

servo_data_id1_to = struct.pack("<BhHB", 1, -4000, 4094, 50)
servo_data_id2_to = struct.pack("<BhHB", 2, -4000, 4094, 50)

servo_data_id1_from = struct.pack("<BhHB", 1, 4500, 4094, 50)
servo_data_id2_from = struct.pack("<BhHB", 2, 4500, 4094, 50)

# connect to Arduino
ard_device = ArduinoConnection("/dev/ttyUSB0", 115200)

while True:
    ard_device.ser.write(servo_data_id1_to)
    time.sleep(2)
    ard_device.ser.write(servo_data_id2_to)
    time.sleep(2)
    ard_device.ser.write(servo_data_id1_from)
    time.sleep(2)
    ard_device.ser.write(servo_data_id2_from)    
    time.sleep(2)

data = list()
print(ard_device.ser.read_until())