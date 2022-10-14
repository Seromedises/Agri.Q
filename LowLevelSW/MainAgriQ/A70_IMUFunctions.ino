void IMU_init(){
  Wire.begin(I2C_MASTER, 0x00, 33, 34);
  Wire.beginTransmission(AgriQFIMU.MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  }

float alpha_IMU_ac = 0.1;
float alpha_IMU_gy = 0.4;
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

    AgriQFIMU.aX = alpha_IMU_ac * AgriQFIMU.AcX * 9.806/16384.0 + (1-alpha_IMU_ac) * AgriQFIMU.aX; // m/s^2
    AgriQFIMU.aY = alpha_IMU_ac * AgriQFIMU.AcY * 9.806/16384.0 + (1-alpha_IMU_ac) * AgriQFIMU.aY; // m/s^2
    AgriQFIMU.aZ = alpha_IMU_ac * AgriQFIMU.AcZ * 9.806/16384.0 + (1-alpha_IMU_ac) * AgriQFIMU.aZ; // m/s^2

    AgriQFIMU.gX = alpha_IMU_gy * AgriQFIMU.GyX * 3.14/(180.0 * 131.0) + (1-alpha_IMU_gy) * AgriQFIMU.gX; // rad/s
    AgriQFIMU.gY = alpha_IMU_gy * AgriQFIMU.GyY * 3.14/(180.0 * 131.0) + (1-alpha_IMU_gy) * AgriQFIMU.gY; // rad/s
    AgriQFIMU.gZ = alpha_IMU_gy * AgriQFIMU.GyZ * 3.14/(180.0 * 131.0) + (1-alpha_IMU_gy) * AgriQFIMU.gZ; // rad/s

    Serial.print( AgriQFIMU.GyX * 9.806/16384.0 );
    Serial.print( ", " );
    Serial.println( AgriQFIMU.gX );


}
