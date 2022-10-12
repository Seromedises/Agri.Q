void TractionMotors_init(){
  pinMode(MFR.PWM_PIN,    OUTPUT);
  analogWriteFrequency(MFR.PWM_PIN, 14e3);
  pinMode(MFL.PWM_PIN,    OUTPUT);
  analogWriteFrequency(MFL.PWM_PIN, 14e3);
  
  pinMode(MRR.PWM_PIN,    OUTPUT); // MRR
  analogWriteFrequency(MRR.PWM_PIN, 14e3);
  //pinMode(MRL.PWM_PIN,    OUTPUT); //NOT connected
   
  analogWrite(MFL.PWM_PIN,2047); //sx 0 avanti //sx 4095 indietro // sx 2047
  analogWrite(MFR.PWM_PIN,2047); //sx 0 avanti //sx 4095 indietro // sx 2047
  analogWrite(MRR.PWM_PIN,2047); //sx 0 avanti //sx 4095 indietro // sx 2047
  
  }

void MFL_MeasureOmega(){
  const float alpha_filter = 0.2;
  MFL.EncCount = Enc_MFL.read();
  MFL.omega_measured = - 2.0*3.14*(MFL.EncCount - MFL.EncCountOLD)/time_sample/MFL.CPR;
  MFL.EncCountOLD = MFL.EncCount;
  MFL.omega_measured = alpha_filter * MFL.omega_measured + (1 - alpha_filter)*MFL.omega_measuredOLD;
  MFL.omega_measuredOLD = MFL.omega_measured;
}

void MFR_MeasureOmega(){
  const float alpha_filter = 0.2;
  
  MFR.EncCount = Enc_MFR.read();
  MFR.omega_measured = - 2.0*3.14*(MFR.EncCount - MFR.EncCountOLD)/time_sample/MFR.CPR;
  MFR.EncCountOLD = MFR.EncCount;
  MFR.omega_measured = alpha_filter * MFR.omega_measured + (1 - alpha_filter)*MFR.omega_measuredOLD;
  MFR.omega_measuredOLD = MFR.omega_measured;
}

void MRL_MeasureOmega(){
  const float alpha_filter = 0.2;
  
  MRL.EncCount = Enc_MRL.read();
  MRL.omega_measured = 2.0*3.14*(MRL.EncCount - MRL.EncCountOLD)/time_sample/MRL.CPR;
  MRL.EncCountOLD = MRL.EncCount;
  MRL.omega_measured = alpha_filter * MRL.omega_measured + (1 - alpha_filter)*MRL.omega_measuredOLD;
  MRL.omega_measuredOLD = MRL.omega_measured;
}

void MRR_MeasureOmega(){
  const float alpha_filter = 0.2;
  
  MRR.EncCount = Enc_MRR.read();
  MRR.omega_measured = 2.0*3.14*(MRR.EncCount - MRR.EncCountOLD)/time_sample/MRR.CPR;
  MRR.EncCountOLD = MRR.EncCount;
  MRR.omega_measured = alpha_filter * MRR.omega_measured + (1 - alpha_filter)*MRR.omega_measuredOLD;
  MRR.omega_measuredOLD = MRR.omega_measured;
}


void MFL_MeasureCurrent(){
  MFL.current_measured = analogRead(MFL.Current_PIN);
  MFL.current_measured = mapfloat(MFL.current_measured, 0, 1024, -20.0,20.0);
}

void MFR_MeasureCurrent(){
  MFR.current_measured = analogRead(MFR.Current_PIN);
  MFR.current_measured = mapfloat(MFR.current_measured, 0, 1024, -20.0,20.0);
}

void MRL_MeasureCurrent(){
  MRL.current_measured = analogRead(MRL.Current_PIN);
  MRL.current_measured = mapfloat(MRL.current_measured, 0, 1024, -20.0,20.0);
}

void MRR_MeasureCurrent(){
  MRR.current_measured = analogRead(MRR.Current_PIN);
  MRR.current_measured = mapfloat(MRR.current_measured, 0, 1024, -20.0,20.0);
}


void MFront_ReferenceOmega(float lambda, float nu, float v_cubic, float phid_cubic){
  if (lambda > lambda_dead){ // Driving forward
    if (phid_cubic >= 0){ // Driving straight or turning left (right side is faster)
      MFR.reference = MFR.tau * (2 * v_cubic + phid_cubic * track)/(2 * r_wheel);
      MFR.reference = constrain(MFR.reference, -omega_max, omega_max);
      MFL.reference = MFR.reference - MFL.tau * phid_cubic * track / r_wheel;
    }
    else if (phid_cubic < 0){ // Turning right (left side is faster)
      MFL.reference = MFL.tau * (2 * v_cubic - phid_cubic * track)/(2 * r_wheel);
      MFL.reference = constrain(MFL.reference, -omega_max, omega_max);
      MFR.reference = MFL.reference + MFR.tau * phid_cubic * track / r_wheel;
    }
  }
  else if (lambda < -lambda_dead){ // Driving backward
    phid_cubic = - phid_cubic;
    if (phid_cubic <= 0){ // Driving straight or turning left (right side is faster)
      MFR.reference = MFR.tau * (2 * v_cubic - phid_cubic * track)/(2 * r_wheel);
      MFR.reference = constrain(MFR.reference, -omega_max, omega_max);
      MFL.reference = MFR.reference + MFL.tau * phid_cubic * track / r_wheel;
    }
    else if (phid_cubic > 0){ // Turning right (left side is faster)
      MFL.reference = MFL.tau * (2 * v_cubic + phid_cubic * track)/(2 * r_wheel);
      MFL.reference = constrain(MFL.reference, -omega_max, omega_max);
      MFR.reference = MFL.reference - MFR.tau * phid_cubic * track / r_wheel;
    }
  }
  else { // Turing on the spot (right and left side are equal and opposite)
      MFR.reference = MFR.tau * (phid_cubic * track)/(2 * r_wheel) * ( pow(lambda/lambda_dead,3) - 3/2 * pow(lambda/lambda_dead,2) + 3/2);
      MFR.reference = constrain(MFR.reference, -omega_max, omega_max);
      MFL.reference = -MFR.reference;
    
  }
 
}

void MFL_outcmd(uint16_t pwm_cmd){
  analogWrite(MFL.PWM_PIN,pwm_cmd); //sx 0 avanti //sx 4095 indietro // sx 2047
}

void MFR_outcmd(uint16_t pwm_cmd){
  analogWrite(MFR.PWM_PIN,pwm_cmd); //sx 0 avanti //sx 4095 indietro // sx 2047
}

void MRL_outcmd(uint16_t pwm_cmd){
  analogWrite(MRR.PWM_PIN,pwm_cmd); //sx 0 avanti //sx 4095 indietro // sx 2047
}

void MRR_outcmd(uint16_t pwm_cmd){
  analogWrite(MRR.PWM_PIN,pwm_cmd); //sx 0 avanti //sx 4095 indietro // sx 2047
}
