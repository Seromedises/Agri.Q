float Yaw_getmeasure(){

  // max sterzo a sx 745, max sterzo a dx 176
  float k = 0.9;
  
  return Yaw_measured = (1-k)* + k*mapfloat(analogRead(Yaw_measuredPin), 176, 745, -40.0 *3.14/180.0, 40.0 *3.14/180.0); //rad
}
