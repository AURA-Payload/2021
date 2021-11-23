/*
The sensor outputs provided by the library are the raw
16-bit values obtained by concatenating the 8-bit high and
low accelerometer and gyro data registers. They can be
converted to units of g and dps (degrees per second) using
the conversion factors specified in the datasheet for your
particular device and full scale setting (gain).

Example: An LSM6DS33 gives an accelerometer Z axis reading
of 16276 with its default full scale setting of +/- 2 g. The
LA_So specification in the LSM6DS33 datasheet (page 11)
states a conversion factor of 0.061 mg/LSB (least
significant bit) at this FS setting, so the raw reading of
16276 corresponds to 16276 * 0.061 = 992.8 mg = 0.9928 g.
*/

#include <Wire.h>
#include <LSM6.h>

LSM6 imu;

char report[80];

void setup()
{
  Serial.begin(9600);
  Wire.begin();

  if (!imu.init())
  {
    Serial.println("Failed to detect and initialize IMU!");
    while (1);
  }
  imu.enableDefault();

  // ODR_XL = 1000 (1.66 kHz (high performance))
  // FS_XL = 01 (+/-16 g full scale)
  // BW_XL = 00 (400 Hz) (Overridden when XL_BW_SCAL_ODR=0)
  imu.writeReg(LSM6::CTRL1_XL, 0b10000100);
  // ODR = 1000 (1.66 kHz (high performance))
  // FS_G = 11 (2000 dps)
  imu.writeReg(LSM6::CTRL2_G, 0b10001100);
}

void loop()
{
  imu.read();

  // Uncomment these for all values
//  snprintf(report, sizeof(report), "A: %6d %6d %6d    G: %6d %6d %6d",
//    imu.a.x, imu.a.y, imu.a.z,
//    imu.g.x, imu.g.y, imu.g.z);
//  Serial.println(report);

  // Uncomment these for acceleration values
  snprintf(report, sizeof(report), "%6d %6d %6d",
    imu.a.x, imu.a.y, imu.a.z);
  Serial.println(report);

  // Uncomment these for gyro values
//  snprintf(report, sizeof(report), "%6d %6d %6d",
//    imu.g.x, imu.g.y, imu.g.z);
//  Serial.println(report);

  delay(100);
}
