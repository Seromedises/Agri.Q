void Pitch_init(){
  pinMode(DirPitchPin,    OUTPUT);
  pinMode(BrakePitchPin,    OUTPUT);
  pinMode(EnPitchPin,    OUTPUT);
  }

void Pitch_outcmd(uint16_t RxPitch){

  bool EnPitch = 0;
  bool DirPitch = 0;
  bool BrakePitch = 0;

  const uint16_t zero_value = 1500;
  const uint16_t halfdead_band_brk = 80;
  const uint16_t halfdead_band = 250;

    if(RxPitch >= zero_value + halfdead_band){
      EnPitch = 1; DirPitch = 0; BrakePitch = 1;
      }
    if(RxPitch >= zero_value + halfdead_band_brk){
      DirPitch = 0; BrakePitch = 1;
      }
     if(RxPitch > zero_value - halfdead_band_brk && RxPitch < zero_value + halfdead_band_brk){
      //BrakePitch = 0; 
      EnPitch = 0;DirPitch = 1;
     }
    if(RxPitch <= zero_value - halfdead_band_brk){
      DirPitch = 1; BrakePitch = 1; EnPitch = 0;
      }
    if(RxPitch <= zero_value - halfdead_band){
      EnPitch = 1; DirPitch = 1; BrakePitch = 1;
      }

  digitalWrite(DirPitchPin, DirPitch);
  digitalWrite(EnPitchPin, EnPitch);
  digitalWrite(BrakePitchPin, BrakePitch ); 

}

float Pitch_getmeasure(){
  // max pitch back , max pitch front
  return mapfloat(analogRead(Pitch_measuredPin), 578, 287, -5.44 *3.14/180.0, 33.0 *3.14/180.0); //rad
}
