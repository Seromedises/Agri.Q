// RX INITIALISATION
void Rx_init(){
 pinMode( AgriQJoyRx.PIN_J1updwn, INPUT_PULLUP);
 attachInterrupt(digitalPinToInterrupt(AgriQJoyRx.PIN_J1updwn), rx_J1updown,  CHANGE);

 pinMode( AgriQJoyRx.PIN_J1leftright, INPUT_PULLUP);
 attachInterrupt(digitalPinToInterrupt(AgriQJoyRx.PIN_J1leftright), rx_J1leftright,  CHANGE);

  pinMode( AgriQJoyRx.PIN_J2updwn, INPUT_PULLUP);
 attachInterrupt(digitalPinToInterrupt(AgriQJoyRx.PIN_J2updwn), rx_J2updown,  CHANGE);

 pinMode( AgriQJoyRx.PIN_J2leftright, INPUT_PULLUP);
 attachInterrupt(digitalPinToInterrupt(AgriQJoyRx.PIN_J2leftright), rx_J2leftright,  CHANGE);

 pinMode( AgriQJoyRx.PIN_SW1, INPUT_PULLUP);
 attachInterrupt(digitalPinToInterrupt(AgriQJoyRx.PIN_SW1), rx_SW1,  CHANGE);

 pinMode( AgriQJoyRx.PIN_SW2, INPUT_PULLUP);
 attachInterrupt(digitalPinToInterrupt(AgriQJoyRx.PIN_SW2), rx_SW2,  CHANGE);
}

// RX CALLBACK FUNTIONS
void rx_J1updown(){
    if(digitalRead(AgriQJoyRx.PIN_J1updwn)){
      AgriQJoyRx.timer_J1updwn = micros();
    }
    else {
      AgriQJoyRx.VALUE_J1updwn = (uint16_t) (micros() - AgriQJoyRx.timer_J1updwn);
    } 
  }
  
void rx_J1leftright(){
    if(digitalRead(AgriQJoyRx.PIN_J1leftright)){
      AgriQJoyRx.timer_J1leftright = micros();
    }
    else {
      AgriQJoyRx.VALUE_J1leftright = (uint16_t) (micros() - AgriQJoyRx.timer_J1leftright);
    }
  }

void rx_J2updown(){
    if(digitalRead(AgriQJoyRx.PIN_J2updwn)){
      AgriQJoyRx.timer_J2updwn = micros();
    }
    else {
      AgriQJoyRx.VALUE_J2updwn = (uint16_t) (micros() - AgriQJoyRx.timer_J2updwn);
    }
  }
  
void rx_J2leftright(){
    if(digitalRead(AgriQJoyRx.PIN_J2leftright)){
      AgriQJoyRx.timer_J2leftright = micros();
    }
    else {
      AgriQJoyRx.VALUE_J2leftright = (uint16_t) (micros() - AgriQJoyRx.timer_J2leftright);
    }
}

void rx_SW1(){
    if(digitalRead(AgriQJoyRx.PIN_SW1)){
      AgriQJoyRx.timer_SW1 = micros();
    }
    else {
      AgriQJoyRx.VALUE_SW1 = (uint16_t) (micros() - AgriQJoyRx.timer_SW1);
    }
  }
  
  void rx_SW2(){
    if(digitalRead(AgriQJoyRx.PIN_SW2)){
      AgriQJoyRx.timer_SW2 = micros();
    }
    else {
      AgriQJoyRx.VALUE_SW2 = (uint16_t) (micros() - AgriQJoyRx.timer_SW2);
    }
  }
