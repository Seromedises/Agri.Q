void EnBrake_init(){
  pinMode(EnableFrontMotorsPin,    OUTPUT);
  pinMode(BrakeFrontMotorsPin,    OUTPUT);
  pinMode(EnableRearMotorsPin,    OUTPUT);
  pinMode(BrakeRearMotorsPin,    OUTPUT);

  // Disable motors
  digitalWrite(EnableFrontMotorsPin, 0);
  digitalWrite(EnableRearMotorsPin, 0);
  // Activate brakes
  digitalWrite(BrakeFrontMotorsPin, 0);
  digitalWrite(BrakeRearMotorsPin, 0);
    
  }

  bool EnBrake_outcmd(uint16_t RxSw1, uint16_t RxSw2){


  const uint16_t zero_value = 1500;
  const uint16_t halfdead_band = 100;
 
  if(RxSw1 >= zero_value + halfdead_band){
    BrakeFrontMotors = 1;
    BrakeRearMotors = 1;
    StateAuto = 1;
    }
   if(RxSw1 > zero_value - halfdead_band && RxSw1 < zero_value + halfdead_band){
    BrakeFrontMotors = 1;
    BrakeRearMotors = 1;
    StateAuto = 0;
   }
  if(RxSw1 <= zero_value - halfdead_band){
    BrakeFrontMotors = 0;
    BrakeRearMotors = 0;
    }
////////////
  if(RxSw2 >= zero_value + halfdead_band){
    EnableFrontMotors = 1;
    EnableRearMotors = 1;
    }
   if(RxSw2 > zero_value - halfdead_band && RxSw2 < zero_value + halfdead_band){
    EnableFrontMotors = 1;
    EnableRearMotors = 0;
    }
  if(RxSw2 <= zero_value - halfdead_band){
    EnableFrontMotors = 0;
    EnableRearMotors = 0;
    }
    
  digitalWrite(EnableFrontMotorsPin, EnableFrontMotors);
  digitalWrite(BrakeFrontMotorsPin, BrakeFrontMotors);
  digitalWrite(EnableRearMotorsPin, EnableRearMotors);
  digitalWrite(BrakeRearMotorsPin, BrakeRearMotors);

    
  return StateAuto;
}
