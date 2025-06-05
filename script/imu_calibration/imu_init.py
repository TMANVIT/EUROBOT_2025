import serial
import time
import math
import numpy as np
import struct
from array import array

# Set the correct serial port parameters------------------------
ser_port = "/dev/ttyUSB0"     # Adjust to /dev/ttyUSB0 for Linux if needed
ser_baudrate = 115200 # Serial port baud rate
ser_timeout = 2       # Serial port operation timeout time

# Open the serial port
ser = serial.Serial(ser_port, ser_baudrate, timeout=ser_timeout)

# Configuration parameters
frequency_hz = 250     # Desired sensor data transmission frequency in Hz (0-250, 0 means 0.5 Hz)
set_frequency_only = True  # Flag: True to only set frequency, False to perform full initialization

flag = 0

def Cmd_RxUnpack(buf, DLen):
    global flag

    scaleAccel       = 0.00478515625
    scaleQuat        = 0.000030517578125
    scaleAngle       = 0.0054931640625
    scaleAngleSpeed  = 0.06103515625
    scaleMag         = 0.15106201171875
    scaleTemperature = 0.01
    scaleAirPressure = 0.0002384185791
    scaleHeight      = 0.0010728836

    if buf[0] == 0x11:
        ctl = (buf[2] << 8) | buf[1]
        print("\n subscribe tag: 0x%04x"%ctl)
        print(" ms: ", ((buf[6]<<24) | (buf[5]<<16) | (buf[4]<<8) | (buf[3]<<0)))

        L = 7 # Starting from the 7th byte, the remaining data is parsed according to the subscription identification tag.
        if ((ctl & 0x0001) != 0):  
            tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2 
            print("\taX: %.3f"%tmpX) # Acceleration ax without gravity
            tmpY = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2 
            print("\taY: %.3f"%tmpY) # Acceleration ay without gravity
            tmpZ = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2
            print("\taZ: %.3f"%tmpZ) # Acceleration az without gravity
                    
        if ((ctl & 0x0002) != 0):
            # Optional calibration offsets (uncomment if needed)
            # AX_bias = 0.086  # Adjust based on measurements
            # AY_bias = -0.033  # Adjust based on measurements
            tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2
            # tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel - AX_bias; L += 2
            print("\tAX: %.3f"%tmpX) # Acceleration AX with gravity
            tmpY = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2
            # tmpY = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel - AY_bias; L += 2
            print("\tAY: %.3f"%tmpY) # Acceleration AY with gravity
            tmpZ = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2
            print("\tAZ: %.3f"%tmpZ) # Acceleration AZ with gravity
            tmpAbs = np.sqrt(tmpX*tmpX + tmpY*tmpY + tmpZ*tmpZ)
            print("\tAbs: %.3f"%tmpAbs) # Acceleration module

        if ((ctl & 0x0004) != 0):
            tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAngleSpeed; L += 2 
            print("\tGX: %.3f"%tmpX) # Angular velocity GX
            tmpY = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAngleSpeed; L += 2 
            print("\tGY: %.3f"%tmpY) # Angular velocity GY
            tmpZ = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAngleSpeed; L += 2
            print("\tGZ: %.3f"%tmpZ) # Angular velocity GZ
        
        if ((ctl & 0x0008) != 0):
            tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleMag; L += 2
            print("\tCX: %.3f"%tmpX) # Magnetic field data CX
            tmpY = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleMag; L += 2
            print("\tCY: %.3f"%tmpY) # Magnetic field data CY
            tmpZ = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleMag; L += 2
            print("\tCZ: %.3f"%tmpZ) # Magnetic field data CZ
            tmpAbs = math.sqrt(math.pow(tmpX,2) + math.pow(tmpY,2) + math.pow(tmpZ,2))
            print("\tCAbs: %.3f"%tmpAbs) # Absolute value of 3-axis composite
        
        if ((ctl & 0x0010) != 0):
            tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleTemperature; L += 2
            print("\ttemperature: %.2f"%tmpX) # temperature
            tmpU32 = np.uint32(((np.uint32(buf[L+2]) << 16) | (np.uint32(buf[L+1]) << 8) | np.uint32(buf[L])))
            if ((tmpU32 & 0x800000) == 0x800000):
                tmpU32 = (tmpU32 | 0xff000000)      
            tmpY = np.int32(tmpU32) * scaleAirPressure; L += 3
            print("\tairPressure: %.3f"%tmpY) # air pressure
            tmpU32 = np.uint32((np.uint32(buf[L+2]) << 16) | (np.uint32(buf[L+1]) << 8) | np.uint32(buf[L]))
            if ((tmpU32 & 0x800000) == 0x800000):
                tmpU32 = (tmpU32 | 0xff000000)
            tmpZ = np.int32(tmpU32) * scaleHeight; L += 3 
            print("\theight: %.3f"%tmpZ) # height

        if ((ctl & 0x0020) != 0):
            tmpAbs = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleQuat; L += 2
            print("\tw: %.3f"%tmpAbs) # Quaternions w
            tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleQuat; L += 2
            print("\tx: %.3f"%tmpX) # Quaternions x
            tmpY = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleQuat; L += 2
            print("\ty: %.3f"%tmpY) # Quaternions y
            tmpZ = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleQuat; L += 2
            print("\tz: %.3f"%tmpZ) # Quaternions z

        if ((ctl & 0x0040) != 0):
            tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAngle; L += 2
            print("\tangleX: %.3f"%tmpX) # Euler angles x 
            tmpY = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAngle; L += 2
            print("\tangleY: %.3f"%tmpY) # Euler angles y 
            tmpZ = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAngle; L += 2
            print("\tangleZ: %.3f"%tmpZ) # Euler angles z
            
        flag = 1
    elif buf[0] == 0x34:
        print("\taccelRange: %d"%buf[1]) # accelRange 
        print("\tgyroRange: %d"%buf[2]) # gyroRange 
    else:
        print(f"------data head not defined: 0x{buf[0]:02X}")

CmdPacket_Begin = 0x49   # Start code
CmdPacket_End = 0x4D     # End code
CmdPacketMaxDatSizeRx = 73  # Maximum length of the data body

CS = 0  # Checksum
i = 0
RxIndex = 0
buf = bytearray(5 + CmdPacketMaxDatSizeRx) # Receive packet cache
cmdLen = 0 # Length

def Cmd_GetPkt(byte):
    global CS, i, RxIndex, buf, cmdLen
    CS += byte
    if RxIndex == 0: # Start code
        if byte == CmdPacket_Begin:
            i = 0
            buf[i] = CmdPacket_Begin
            i += 1
            CS = 0
            RxIndex = 1
    elif RxIndex == 1: # Address code
        buf[i] = byte
        i += 1
        if byte == 255:
            RxIndex = 0
        else:
            RxIndex += 1
    elif RxIndex == 2: # Data body length
        buf[i] = byte
        i += 1
        if byte > CmdPacketMaxDatSizeRx or byte == 0:
            RxIndex = 0
        else:
            RxIndex += 1
            cmdLen = byte
    elif RxIndex == 3: # Data body
        buf[i] = byte
        i += 1
        if i >= cmdLen + 3:
            RxIndex += 1
    elif RxIndex == 4: # Checksum
        CS -= byte
        if (CS & 0xFF) == byte:
            buf[i] = byte
            i += 1
            RxIndex += 1
        else:
            RxIndex = 0
    elif RxIndex == 5: # End code
        RxIndex = 0
        if byte == CmdPacket_End:
            buf[i] = byte
            i += 1
            hex_string = " ".join(f"{b:02X}" for b in buf[0:i])
            print(f"U-Rx[Len={i}]:{hex_string}")
            Cmd_RxUnpack(buf[3:i-2], i-5)
            return 1
        else:
            print(f"Invalid end code: 0x{byte:02X}")
    else:
        print(f"Invalid packet, byte: 0x{byte:02X}, RxIndex: {RxIndex}")
        RxIndex = 0
    return 0

def Cmd_PackAndTx(pDat, DLen):
    if DLen == 0 or DLen > 19:
        return -1
    buf = bytearray([0x00]*46) + bytearray([0x00, 0xff, 0x00, 0xff, 0x49, 0xFF, DLen]) + bytearray(pDat[:DLen])
    CS = sum(buf[51:51+DLen+2]) & 0xFF
    buf.append(CS)
    buf.append(0x4D)
    ser.write(buf)
    return 0

def read_data():
    global flag
    flag = 0
    print("------------Start--------------")

    # Parameter settings
    isCompassOn = 0 # Magnetic field fusion: 0=Not used, 1=Used
    barometerFilter = 2
    Cmd_ReportTag = 0x02 # Feature subscription tag
    params = bytearray([0x00 for i in range(0,11)])
    params[0] = 0x12
    params[1] = 7       # Stationary state acceleration threshold
    params[2] = 255     # Static zero return speed (cm/s): 0=No return, 255=Immediate
    params[3] = 0       # Dynamic zero return speed (cm/s): 0=No return
    params[4] = ((barometerFilter&3)<<1) | (isCompassOn&1)
    params[5] = frequency_hz if frequency_hz <= 250 else 250  # Data transmission frequency [0-250 Hz], 0=0.5 Hz
    params[6] = 1       # Gyroscope filter coefficient [0-2]
    params[7] = 1       # Accelerometer filter coefficient [0-4]
    params[8] = 5       # Magnetometer filter coefficient [0-9]
    params[9] = Cmd_ReportTag&0xff
    params[10] = (Cmd_ReportTag>>8)&0xff    
    Cmd_PackAndTx(params, len(params))
    time.sleep(0.2)

    # Wake up sensor
    Cmd_PackAndTx([0x03], 1)
    time.sleep(0.2)

    if not set_frequency_only:
        # Disable proactive reporting
        Cmd_PackAndTx([0x18], 1)

        # Set accelerometer and gyroscope range
        # AccRange: 0=2g, 1=4g, 2=8g, 3=16g
        # GyroRange: 0=256, 1=512, 2=1024, 3=2048
        Cmd_PackAndTx([0x33, 0x00, 0x00], 3)

        # Start accelerometer calibration
        print("------------Start accelerometer calibration--------------")
        Cmd_PackAndTx([0x17, 0x00], 2)
        print("------------Static automatic collection of data, the light does not turn on, it means that this surface is collected, the light will resume flashing, waiting for the next surface to be collected, to collect six faces in total.--------------")
        input("Press Enter to continue...")
 
        # Finish accelerometer calibration
        print("------------Finish accelerometer calibration--------------")   
        Cmd_PackAndTx([0x17, 0xff], 2)

        # Read accelerometer and gyroscope range
        print("------------Read the range of accelerometer and gyroscope--------------")  
        Cmd_PackAndTx([0x34], 1)

    # Read data once
    Cmd_PackAndTx([0x11], 1)
    print(f"------------Sensor data requested, frequency set to {frequency_hz} Hz--------------")

    # Loop to receive data
    while flag == 0:
        data = ser.read(1)
        if len(data) > 0:
            Cmd_GetPkt(data[0])

# Start reading data
read_data()
print("------------Finish--------------")