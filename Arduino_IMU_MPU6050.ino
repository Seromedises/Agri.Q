// MPU-6050 Short Example Sketch
#include<Wire.h>
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
float aX = 0.0,aY = 0.0,aZ = 0.0,TmpC = 0.0,gX = 0.0,gY = 0.0,gZ = 0.0;
void setup(){
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(57600);

  analogWriteResolution(16); // 16 bit PWM
  
}

int tempo0 = millis();
unsigned long tempo = 0;

void loop(){

  tempo = millis()-tempo0;
  
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  
  
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  /*
  analogWrite(3, AcX + 32768);
  analogWrite(4, AcY + 32768);
  analogWrite(5, AcZ + 32768);
  analogWrite(23, GyX + 32768);
  analogWrite(22, GyY + 32768);
  analogWrite(21, GyZ + 32768);
  aX = EWMA_filter(0.25,aX,fmap(AcX,-15300,17400,-9.81,9.81));
  aY = EWMA_filter(0.25,aY,fmap(AcY,-16600,16200,-9.81,9.81));
  aZ = EWMA_filter(0.25,aZ,fmap(AcZ,-19900,13800,-9.81,9.81));
  */
  
  // Serial.print(F("AcX = ")); 
  Serial.print(tempo);
  Serial.print(",");
  Serial.print(AcX);
  Serial.print(",");
  // Serial.print(F(" | AcY = ")); 
  Serial.print(AcY);
  Serial.print(",");
  // Serial.print(" | AcZ = "); 
  Serial.print(AcZ);
  Serial.print(",");
  // Serial.print(" | Tmp = "); 
  // Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
  //Serial.print(" | GyX = "); 
  Serial.print(GyX);
  Serial.print(",");
  //Serial.print(" | GyY = "); 
  Serial.print(GyY);
  Serial.print(",");
  //Serial.print(" | GyZ = "); 
  Serial.println(GyZ);
  delay(20);
  //Serial.println();
}


float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

////////////////////////////////////////
//// EWMA Filter (digital low pass) ////
float EWMA_filter(float alfa, float measure_filt, float measure_now) {
  return measure_filt = (1 - alfa) * measure_filt + alfa * measure_now;
}
