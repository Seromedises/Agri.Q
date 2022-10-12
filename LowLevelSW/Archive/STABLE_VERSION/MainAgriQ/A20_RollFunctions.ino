void Roll_init(){
  pinMode(DirRollPin,    OUTPUT);
  pinMode(PWMRollPin,    OUTPUT);
  pinMode(EnRollPin,    OUTPUT);
  }

void Roll_outcmd(uint16_t RxRoll){

  bool EnRoll = 0;
  bool DirRoll = 0;
  bool PWMRoll = 0;

  const uint16_t zero_value = 1500;
  const uint16_t halfdead_band = 150;

  EnRoll = 0;

  if(RxRoll >= zero_value + halfdead_band){
    DirRoll = 0; PWMRoll = 1; EnRoll = 1;
    }
  if(RxRoll > zero_value - halfdead_band && RxRoll < zero_value + halfdead_band){
    EnRoll = 0; PWMRoll = 0;
   }
  if(RxRoll <= zero_value - halfdead_band){
    DirRoll = 1; PWMRoll = 1; EnRoll = 1;
    }
    
  digitalWrite(DirRollPin, DirRoll);
  digitalWrite(PWMRollPin, PWMRoll);
  digitalWrite(EnRollPin, EnRoll);

}

float Roll_getmeasure(){
  // max roll sx 310, max roll dx 221
  return mapfloat(analogRead(Roll_measuredPin), 310, 221, -20.0 *3.14/180.0, 20.0 *3.14/180.0); //rad
}


uint16_t RollBack_getmeasure(){
  return analogRead(Roll_measuredPin); //ADD CONVERSION FROM BITS TO ANGLE (change to float, add filter?)
}
