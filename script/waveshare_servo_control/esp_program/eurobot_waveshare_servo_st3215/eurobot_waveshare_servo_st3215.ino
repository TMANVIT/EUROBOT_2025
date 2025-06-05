/*
  The normal write example passed the test in ST3215 Servo,
  and if testing other models of ST series servos
  please change the appropriate position, speed and delay parameters.
*/

#include <SCServo.h>

SMS_STS st;

// the uart used to control servos.
// GPIO 18 - S_RXD, GPIO 19 - S_TXD, as default.
#define S_RXD 18
#define S_TXD 19

void setup()
{
  Serial.begin(115200);
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  st.pSerial = &Serial1;
  delay(1000);
}

void loop()
{
  if (Serial.available()) {
    byte servo_data[6];
    for (int i = 0; i < 6; i++) {
      servo_data[i] = Serial.read();
    }
    uint8_t servo_id = servo_data[0];
    int16_t servo_position = (int16_t)(servo_data[1]) + (int16_t)(servo_data[2] << 8);
    uint16_t servo_speed = (uint16_t)servo_data[3] | (uint16_t)(servo_data[4] << 8);
    uint8_t servo_acc = servo_data[5];
    
    // Debug
        Serial.print(servo_id);
        Serial.print(" ");
        Serial.print(servo_position);
        Serial.print(" ");
        Serial.print(servo_speed);
        Serial.print(" ");
        Serial.print(servo_acc);
        Serial.println();
    
    // message format: SevoID(1 byte), Position(2 bytes), Speed(2 bytes), Acceleration(1 byte)
    // int SCSCL::WritePosEx(u8 ID, s16 Position, u16 Speed, u8 ACC)
    
    st.WritePosEx(servo_id, servo_position, servo_speed, servo_acc);
  }



}
