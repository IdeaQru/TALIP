#include "Sensor/Gyro.h"
unsigned char Re_buf[8], counter = 0;
void GYRO :: init() {

  // Kalibrasi Heading
  // Serial2.write(0xA5);
  // Serial2.write(0x55);
  //     // Kalibrasi Tilt
  // delay(4000);
  Serial2.write(0XA5);
  Serial2.write(0X54);//correction mode
  delay(4000);
  // Kalibrasi Heading
  Serial2.write(0xA5);
  Serial2.write(0x55);
  delay(4000);
  Serial2.write(0XA5);
  Serial2.write(0X51);
}
void GYRO :: read() {
  Serial2.write(0XA5);
  Serial2.write(0X51);//send it for each read
  while (Serial2.available()) {
    Re_buf[counter] = (unsigned char)Serial2.read();
    if (counter == 0 && Re_buf[0] != 0xAA) return;
    counter++;
    if (counter == 8)
    {
      counter = 0;
      if (Re_buf[0] == 0xAA && Re_buf[7] == 0x55) // data package is correct
      {
        Yaw = (int16_t)(Re_buf[1] << 8 | Re_buf[2]) / 100.00;
        Pitch = (int16_t)(Re_buf[3] << 8 | Re_buf[4]) / 100.00;
        Roll = (int16_t)(Re_buf[5] << 8 | Re_buf[6]) / 100.00;
      }
    }

  }
  // Serial.println(Roll);
}


float GYRO :: getYaw() {
    return Yaw;
}
float GYRO :: getRoll() {
    return Roll;
}
float GYRO :: getPitch() {
    return Pitch;
}