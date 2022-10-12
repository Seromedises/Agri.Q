void measurePanel(){
  
  AgriQPower.PanelCurrent = analogRead(PanelCurrentPin);
  //AgriQPower.PanelCurrent = (3.3* AgriQPower.m_PanelCurrent * AgriQPower.PanelCurrent/1024) + AgriQPower.q_PanelCurrent;
  
  AgriQPower.PanelVoltage = analogRead(PanelVoltagePin);
  //AgriQPower.PanelVoltage = AgriQPower.m_PanelVoltage * AgriQPower.PanelVoltage + AgriQPower.q_PanelVoltage;

  //Serial.print(AgriQPower.PanelCurrent); Serial.print(",");
  //Serial.println(AgriQPower.PanelVoltage);
    
}

void measureBattery(){
  
  AgriQPower.BatteryCurrent = analogRead(BatteryCurrentPin);
  //AgriQPower.BatteryCurrent = (5.0* AgriQPower.m_BatteryCurrent * AgriQPower.BatteryCurrent/1024) + AgriQPower.q_BatteryCurrent;
  
  AgriQPower.BatteryVoltage = analogRead(BatteryVoltagePin);
  //AgriQPower.BatteryVoltage = AgriQPower.m_BatteryVoltage * AgriQPower.BatteryVoltage + AgriQPower.q_BatteryVoltage;


  //Serial.print(AgriQPower.BatteryCurrent); Serial.print(",");
  //Serial.println(AgriQPower.BatteryVoltage);
}
