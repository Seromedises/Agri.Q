float Yaw_getmeasure(){

  // max sterzo a sx 745, max sterzo a dx 176
  return mapfloat(analogRead(Yaw_measuredPin), 176, 745, -40.0 *3.14/180.0, 40.0 *3.14/180.0); //rad
}
