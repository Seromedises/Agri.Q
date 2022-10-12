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

// Function to receive panel pitch and roll reference value
// the expected string is in the form "<xx.xx, yy.yy>", 
// where xx.xx is the pitch reference value, and yy.yy is the reference roll value
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
                
                PanelAutoRoll = 1;
                PanelAutoPitch = 1;
                AgriQAutoAdvance = 1;
                
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

// Function to extract the two float number from the string
void parseData() {      // split the data into its parts

    char * strtokIndx; // this is used by strtok() as an index

    strtokIndx = strtok(tempChars,",");      // get the first part - the string
    Pitch_reference = atof(strtokIndx);     // convert this part to a float
 
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    Roll_reference = atof(strtokIndx);     // convert this part to a float

    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    Advance_reference = atof(strtokIndx);     // convert this part to a float


}
