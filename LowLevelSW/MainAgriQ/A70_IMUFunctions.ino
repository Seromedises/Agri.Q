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
    AgriQFIMU.aY = -AgriQFIMU.aY; //Align IMU Y Axis to Agri.Q Y axis
    AgriQFIMU.aZ = alpha_IMU_ac * AgriQFIMU.AcZ * g/16384.0 + (1-alpha_IMU_ac) * AgriQFIMU.aZ; // m/s^2
    AgriQFIMU.aZ = -AgriQFIMU.aZ; //Align IMU Z Axis to Agri.Q Z axis

    AgriQFIMU.gX = alpha_IMU_gy * AgriQFIMU.GyX * PI/(180.0 * 131.0) + (1-alpha_IMU_gy) * AgriQFIMU.gX; // rad/s
    AgriQFIMU.gY = alpha_IMU_gy * AgriQFIMU.GyY * PI/(180.0 * 131.0) + (1-alpha_IMU_gy) * AgriQFIMU.gY; // rad/s
    AgriQFIMU.gY = -AgriQFIMU.gY; //Align Gyro Y Axis to Agri.Q Y axis
    AgriQFIMU.gZ = alpha_IMU_gy * AgriQFIMU.GyZ * PI/(180.0 * 131.0) + (1-alpha_IMU_gy) * AgriQFIMU.gZ; // rad/s
    AgriQFIMU.gZ = -AgriQFIMU.gZ; //Align Gyro Z Axis to Agri.Q Z axis


    MadgwickQuaternionUpdate(q, AgriQFIMU.aX, AgriQFIMU.aY, AgriQFIMU.aZ, AgriQFIMU.gX, AgriQFIMU.gY, AgriQFIMU.gZ);

    // SERIAL DEBUG
    Serial.print( q[1] );
    Serial.print( ", " );
    Serial.println( q[2] );

}

  void MadgwickQuaternionUpdate(float q[], float ax, float ay, float az, float gyrox, float gyroy, float gyroz){
    // BASED ON 
    // https://ieeexplore.ieee.org/document/5975346
    // https://nitinjsanket.github.io/tutorials/attitudeest/madgwick
    // https://github.com/kriswiner/MPU6050/blob/master/quaternionFilter.ino


    float deltat = 10 / 1000; // main loop period s
    
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];         // short name local variable for readability
    float norm;                                               // vector norm
    float f1, f2, f3;                                         // objetive funcyion elements
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
    float qDot1, qDot2, qDot3, qDot4;
    float hatDot1, hatDot2, hatDot3, hatDot4;
    float gerrx, gerry, gerrz, gbiasx, gbiasy, gbiasz;        // gyro bias error

    float GyroMeasError = PI * (40.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
    float beta = sqrt(3.0f / 4.0f) * GyroMeasError;  // compute beta
    float GyroMeasDrift = PI * (2.0f / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
    float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

    // Auxiliary variables to avoid repeated arithmetic
    float _halfq1 = 0.5f * q1;
    float _halfq2 = 0.5f * q2;
    float _halfq3 = 0.5f * q3;
    float _halfq4 = 0.5f * q4;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    //float _2q1q3 = 2.0f * q1 * q3;
    //float _2q3q4 = 2.0f * q3 * q4;

    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    ax *= norm;
    ay *= norm;
    az *= norm;
    
    // Compute the objective function and Jacobian
    f1 = _2q2 * q4 - _2q1 * q3 - ax;
    f2 = _2q1 * q2 + _2q3 * q4 - ay;
    f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - az;
    J_11or24 = _2q3;
    J_12or23 = _2q4;
    J_13or22 = _2q1;
    J_14or21 = _2q2;
    J_32 = 2.0f * J_14or21;
    J_33 = 2.0f * J_11or24;
  
    // Compute the gradient (matrix multiplication)
    hatDot1 = J_14or21 * f2 - J_11or24 * f1;
    hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
    hatDot3 = J_12or23 * f2 - J_33 *f3 - J_13or22 * f1;
    hatDot4 = J_14or21 * f1 + J_11or24 * f2;
    
    // Normalize the gradient
    norm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
    hatDot1 /= norm;
    hatDot2 /= norm;
    hatDot3 /= norm;
    hatDot4 /= norm;
    
    // Compute estimated gyroscope biases
    gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
    gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
    gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;
    
    // Compute and remove gyroscope biases
    gbiasx += gerrx * deltat * zeta;
    gbiasy += gerry * deltat * zeta;
    gbiasz += gerrz * deltat * zeta;
    gyrox -= gbiasx;
    gyroy -= gbiasy;
    gyroz -= gbiasz;
    
    // Compute the quaternion derivative
    qDot1 = -_halfq2 * gyrox - _halfq3 * gyroy - _halfq4 * gyroz;
    qDot2 =  _halfq1 * gyrox + _halfq3 * gyroz - _halfq4 * gyroy;
    qDot3 =  _halfq1 * gyroy - _halfq2 * gyroz + _halfq4 * gyrox;
    qDot4 =  _halfq1 * gyroz + _halfq2 * gyroy - _halfq3 * gyrox;

    // Compute then integrate estimated quaternion derivative
    q1 += (qDot1 -(beta * hatDot1)) * deltat;
    q2 += (qDot2 -(beta * hatDot2)) * deltat;
    q3 += (qDot3 -(beta * hatDot3)) * deltat;
    q4 += (qDot4 -(beta * hatDot4)) * deltat;

    // Normalize the quaternion
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    norm = 1.0f/norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;
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
  
  // orientation estimate - To Be Implemented (USE QUATERNIONS)
  IMU_msg.orientation.x = q[1];
  IMU_msg.orientation.y = q[2];
  IMU_msg.orientation.z = q[3];
  IMU_msg.orientation.w = q[0]; // Depends on how a quaternion is defined

  // Set covariance matrices
  for(int i=0;i<9;i++){
        IMU_msg.orientation_covariance[i] = 0; //Unknown  (No data = -1)
        IMU_msg.angular_velocity_covariance[i] = 0; //Unknown
        IMU_msg.linear_acceleration_covariance[i] = 0; //Unknown
        }
  // Set IMU frame
  IMU_msg.header.frame_id = "base_link";
}