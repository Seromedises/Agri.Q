void Pitch_init(){
  pinMode(DirPitchPin,    OUTPUT);
  pinMode(BrakePitchPin,    OUTPUT);
  pinMode(EnPitchPin,    OUTPUT);
  }

bool Pitch_ClosedLoopOutcmd(float Pitch_reference, float Pitch_measured, bool PanelAutoPitch){

  bool EnPitch = 0;
  bool DirPitch = 0;
  bool BrakePitch = 0;

    if(PanelAutoPitch){
      if(abs(Pitch_reference - Pitch_measured)>= 0.0005 ){
        EnPitch = 1; BrakePitch = 1;  DirPitch = ((Pitch_reference - Pitch_measured)< 0);
      }
      else {
        PanelAutoPitch = 0;
        BrakePitch = 0; EnPitch = 0; DirPitch = 0;
      }
    }

  digitalWrite(DirPitchPin, DirPitch);
  digitalWrite(EnPitchPin, EnPitch);
  digitalWrite(BrakePitchPin, BrakePitch ); 

  return PanelAutoPitch;
}

void Pitch_ResetCL(){
  Pitch_reference = Pitch_measured;
  PanelAutoPitch = 0;
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
      EnPitch = 0; DirPitch = 1;
     }
    if(RxPitch <= zero_value - halfdead_band_brk){
      DirPitch = 1; BrakePitch = 1;EnPitch = 0;
      }
    if(RxPitch <= zero_value - halfdead_band){
      EnPitch = 1; DirPitch = 1; BrakePitch = 1;
      }
  digitalWrite(DirPitchPin, DirPitch);
  digitalWrite(EnPitchPin, EnPitch);
  digitalWrite(BrakePitchPin, BrakePitch ); 
}

float Pitch_getmeasure(){
  float k = 0.8;
  
  return (1-k)*Pitch_measured + k*mapfloat(analogRead(Pitch_measuredPin), 578, 287, -5.44 *PI/180.0, 33.0 *PI/180.0); //rad
}
