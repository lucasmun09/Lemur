#include <Wire.h>
#include "mpu6050.h"


// code from https://playground.arduino.cc/Main/MPU-6050
 
void setup()
{      
  int error;
  uint8_t c;
  Serial.begin(9600);
  // Initialize the 'Wire' class ford the I2C-bus.
  Wire.begin();
  
  // According to the datasheet, the 'sleep' bit
  // should read a '1'.
  // That bit has to be cleared, since the sensor
  // is in sleep mode at power-up.
  error = MPU6050_read (MPU6050_PWR_MGMT_1, &c, 1); 
  error = MPU6050_read (MPU6050_PWR_MGMT_1, &c, 1);
  MPU6050_write_reg (MPU6050_PWR_MGMT_1, 0);
}
 
 
void loop()
{
  int error;
  double dT;
  accel_t_gyro_union accel_t_gyro;
 
  // Read the raw values.
  // Read 14 bytes at once,
  // containing acceleration, temperature and gyro.
  // With the default settings of the MPU-6050,
  // there is no filter enabled, and the values
  // are not very stable.
  error = MPU6050_read (MPU6050_ACCEL_XOUT_H, (uint8_t *) &accel_t_gyro, sizeof(accel_t_gyro));
  Serial.print(F("Read accel, temp and gyro, error = "));
  Serial.println(error,DEC);
 
 
  // Swap all high and low bytes.
  // After this, the registers values are swapped,
  // so the structure name like x_accel_l does no
  // longer contain the lower byte.
  uint8_t swap;
 
  SWAP (accel_t_gyro.reg.x_accel_h, accel_t_gyro.reg.x_accel_l);
  SWAP (accel_t_gyro.reg.y_accel_h, accel_t_gyro.reg.y_accel_l);
  SWAP (accel_t_gyro.reg.z_accel_h, accel_t_gyro.reg.z_accel_l);
  SWAP (accel_t_gyro.reg.t_h, accel_t_gyro.reg.t_l);
  SWAP (accel_t_gyro.reg.x_gyro_h, accel_t_gyro.reg.x_gyro_l);
  SWAP (accel_t_gyro.reg.y_gyro_h, accel_t_gyro.reg.y_gyro_l);
  SWAP (accel_t_gyro.reg.z_gyro_h, accel_t_gyro.reg.z_gyro_l);
 
 
  // Print the raw acceleration values
  float x_acc = accel_t_gyro.value.x_accel;
  float y_acc = accel_t_gyro.value.y_accel;
  float z_acc = accel_t_gyro.value.z_accel;

  float total_acc = sqrt(x_acc*x_acc + y_acc*y_acc + z_acc*z_acc);
  Serial.print(F("accel x,y,z,total: "));
  Serial.print(x_acc, 4);
  Serial.print(F(", "));
  Serial.print(y_acc, 4);
  Serial.print(F(", "));
  Serial.print(z_acc, 4);
  Serial.print(F(", "));
  Serial.print(total_acc, 4);
  Serial.println(F(""));
  
 
 
  // The temperature sensor is -40 to +85 degrees Celsius.
  // It is a signed integer.
  // According to the datasheet:
  //   340 per degrees Celsius, -512 at 35 degrees.
  // At 0 degrees: -512 - (340 * 35) = -12412

 /*
  Serial.print(F("temperature: "));
  dT = ( (double) accel_t_gyro.value.temperature + 12412.0) / 340.0;
  Serial.print(dT, 3);
  Serial.print(F(" degrees Celsius"));
  Serial.println(F(""));
 
 
  // Print the raw gyro values.
 
  Serial.print(F("gyro x,y,z : "));
  Serial.print(accel_t_gyro.value.x_gyro, DEC);
  Serial.print(F(", "));
  Serial.print(accel_t_gyro.value.y_gyro, DEC);
  Serial.print(F(", "));
  Serial.print(accel_t_gyro.value.z_gyro, DEC);
  Serial.print(F(", "));
  Serial.println(F(""));
  */
  delay(1000);
}
 
