float mapfloat(float in, float inmin, float inmax, float outmin, float outmax){
  return (in - inmin)*(outmax - outmin)/(inmax - inmin) + outmin;
}

float cubicmapping(float x, float xdead, float xsat, float ymax){
  float y = 0;

  float tempdiv = pow(xdead-xsat,3);
  float a3 = 2/tempdiv;
  float a2 = -3*(xdead + xsat)/tempdiv;
  float a1 = (6*xdead*xsat)/tempdiv;
  float a0 = (pow(xdead,2)*(2*xdead -3*xsat))/((xdead-xsat)*(pow(xdead,2)-2*xdead*xsat + pow(xsat,2)));

  if(abs(x) <= xdead){y = 0.0;}
  if(abs(x) > xdead && abs(x) <= xsat){
    if(x > 0){y = a3*pow(x,3) + a2*pow(x,2) +a1*x +a0;}
    else {y = -(a3*pow(-x,3) + a2*pow(-x,2) +a1*-x +a0);}
  }
  else if(abs(x) > xsat){y = x/abs(x);}
  
  return ymax*y;
}

void GetBaseMeasures(){
  // ANGLES
  Roll_measured = Roll_getmeasure(); // rad Panel roll angle
  Pitch_measured = Pitch_getmeasure(); // rad Panel pitch angle
  RollBack_measured = RollBack_getmeasure();// RAW (integer 0-1024) Roll Back
  Yaw_measured = Yaw_getmeasure(); // rad relative Yaw angle (hitch angle)

  //TRACTION MOTORS - ANGULAR SPEEDS (rad/s) AND CURRENTS (A)
  //// FRONT LEFT
  MFL_MeasureOmega();
  MFL_MeasureCurrent();
  MFL_Odometry();
  //// FRONT RIGHT
  MFR_MeasureOmega();
  MFR_MeasureCurrent();
  MFR_Odometry();
  //// BACK LEFT
  MRL_MeasureOmega();
  MRL_MeasureCurrent();
  //// BACK RIGHT
  MRR_MeasureOmega();
  MRR_MeasureCurrent();

  //BATTERY AND PV PANELS
  measurePanel();
  measureBattery();

  //IMU
  measureIMU();
}

void ResetAllCL(){
  Pitch_ResetCL();
  Roll_ResetCL();
  Advance_ResetCL();

  newData = false;
}


// Function to receive panel pitch and roll reference value
// the expected string is in the form "<xx.xx, yy.yy, zz.zz>", 
// where xx.xx is the pitch reference value, and yy.yy is the reference roll value, and zz.zz is the advance reference
// negative numbers are allowed

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
                
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

// Function to extract the two float number from the string
void parseData(bool SaveSet) {      // split the data into its parts

    char * strtokIndx; // this is used by strtok() as an index
   
    if(SaveSet){
      strtokIndx = strtok(tempChars,",");      // get the first part - the string
      Pitch_reference = atof(strtokIndx);     // convert this part to a float
      PanelAutoPitch = 1;
      strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
      Roll_reference = atof(strtokIndx);     // convert this part to a float
      PanelAutoRoll = 1;
      strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
      Advance_reference = atof(strtokIndx) +0.5*(MFL.odometry + MFR.odometry);     // convert this part to a float
      AgriQAutoAdvance = 1;
    }


}
