import serial
from serial.tools import list_ports


class ArduinoConnection:
    """
    Class for creating a connection with an Arduino via a serial port on Linux.
    """

    def __init__(self, port, speed_connection=9600):
        """
        Parameters
        ----------
        port : str
            Linux serial port for the Arduino (e.g., "/dev/ttyACM0" or "/dev/ttyUSB0")
        speed_connection : int
            Connection speed with the Arduino (default: 9600)
        """
        self.port = port
        self.ser = serial.Serial(port, speed_connection)

    def write(self, value):
        self.ser.write(bytearray(value))#, encoding='utf8'))

    def write_array(self, valuesOfBytes):
        self.ser.write(valuesOfBytes)

    def read(self, size=1):
        return self.ser.read(size=size)


if __name__ == "__main__":
    # On Linux, Arduino devices are typically connected as "/dev/ttyACM0" or "/dev/ttyUSB0".
    port_name = "/dev/ttyUSB0"
    print("Available serial ports:")
    for port in list_ports.comports():
        print(port)
    print()

    ard_device = ArduinoConnection(port_name, 9600)
    while True:
        ch = input("Enter command to send to Arduino: ")
        ard_device.write(ch)
