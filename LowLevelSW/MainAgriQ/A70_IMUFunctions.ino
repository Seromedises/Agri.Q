void IMU_init(){
  Wire.begin();
  Wire.beginTransmission(AgriQFIMU.MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  }


void measureIMU(){
  
    Wire.beginTransmission(AgriQFIMU.MPU_addr);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(AgriQFIMU.MPU_addr,14,true);  // request a total of 14 registers  

    AgriQFIMU.AcX = Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
    AgriQFIMU.AcY = Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AgriQFIMU.AcZ = Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    AgriQFIMU.Tmp = Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    AgriQFIMU.GyX = Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    AgriQFIMU.GyY = Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    AgriQFIMU.GyZ = Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

    AgriQFIMU.aX = AgriQFIMU.AcX/ 16384.0 *9.806; // m/s^2
    AgriQFIMU.aY = AgriQFIMU.AcY/ 16384.0 *9.806; // m/s^2
    AgriQFIMU.aZ = AgriQFIMU.AcZ/ 16384.0 *9.806; // m/s^2

    AgriQFIMU.gX = AgriQFIMU.GyX/ 131.0 * 3.14/180.0; // rad/s
    AgriQFIMU.gY = AgriQFIMU.GyY/ 131.0 * 3.14/180.0; // rad/s
    AgriQFIMU.gZ = AgriQFIMU.GyZ/ 131.0 * 3.14/180.0; // rad/s

}
