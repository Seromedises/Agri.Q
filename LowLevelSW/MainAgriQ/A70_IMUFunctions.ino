void IMU_init(){
  Wire.begin(I2C_MASTER, 0x00, 33, 34); // Set I2C bus to pins 33 and 34
  Wire.beginTransmission(AgriQFIMU.MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  }

float alpha_IMU_ac = 0.1; //Linear acceleration filter parameter (0 - 1)
float alpha_IMU_gy = 0.4; //Angular velocity filter parameter (0 - 1)

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

    //EWMA FILTER

    AgriQFIMU.aX = alpha_IMU_ac * AgriQFIMU.AcX * g/16384.0 + (1-alpha_IMU_ac) * AgriQFIMU.aX; // m/s^2
    AgriQFIMU.aY = alpha_IMU_ac * AgriQFIMU.AcY * g/16384.0 + (1-alpha_IMU_ac) * AgriQFIMU.aY; // m/s^2
    AgriQFIMU.aZ = alpha_IMU_ac * AgriQFIMU.AcZ * g/16384.0 + (1-alpha_IMU_ac) * AgriQFIMU.aZ; // m/s^2

    AgriQFIMU.gX = alpha_IMU_gy * AgriQFIMU.GyX * PI/(180.0 * 131.0) + (1-alpha_IMU_gy) * AgriQFIMU.gX; // rad/s
    AgriQFIMU.gY = alpha_IMU_gy * AgriQFIMU.GyY * PI/(180.0 * 131.0) + (1-alpha_IMU_gy) * AgriQFIMU.gY; // rad/s
    AgriQFIMU.gZ = alpha_IMU_gy * AgriQFIMU.GyZ * PI/(180.0 * 131.0) + (1-alpha_IMU_gy) * AgriQFIMU.gZ; // rad/s

    // SERIAL DEBUG
    //Serial.print( AgriQFIMU.GyX * 9.806/16384.0 );
    //Serial.print( ", " );
    //Serial.println( AgriQFIMU.gX );

}

void buildIMUmsg(){
  // Angular velocity vector
  IMU_msg.angular_velocity.x = AgriQFIMU.gX;
  IMU_msg.angular_velocity.y = AgriQFIMU.gY;
  IMU_msg.angular_velocity.z = AgriQFIMU.gZ;
  // linear acceleration vector
  IMU_msg.linear_acceleration.x = AgriQFIMU.aX;
  IMU_msg.linear_acceleration.y = AgriQFIMU.aY;
  IMU_msg.linear_acceleration.z = AgriQFIMU.aZ;
  
  // No orientation estimate - To Be Implemented (USE QUATERNIONS)
  IMU_msg.orientation.x = 0.0;
  IMU_msg.orientation.y = 0.0;
  IMU_msg.orientation.z = 0.0;
  IMU_msg.orientation.w = 0.0;

  // Set covariance matrices
  for(int i=0;i<9;i++){
        IMU_msg.orientation_covariance[i] = -1; //No data
        IMU_msg.angular_velocity_covariance[i] = 0; //Unknown
        IMU_msg.linear_acceleration_covariance[i] = 0; //Unknown
        }
  // Set IMU frame
  IMU_msg.header.frame_id = "base_link";
}