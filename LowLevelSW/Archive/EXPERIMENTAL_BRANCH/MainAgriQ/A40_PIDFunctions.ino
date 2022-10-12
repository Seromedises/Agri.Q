struct PID_var PID_Ctrl(float ts, float ref, float measure, float last_measure , struct PID_var ij_PID_var, float outMin, float outMax) {
  float kp = ij_PID_var.GainP;
  float ki = ij_PID_var.GainI;
  float kd = ij_PID_var.GainD;
  ts = ts/1000;
  ij_PID_var.error = ref - measure; // error
  ij_PID_var.DER = -kd * (measure - last_measure) / (ts); //Note: the derivative term use just the measure and not the value to avoid derivative kick.
  //In particular, we use -d(measure)/dt. Check theroy for more details.
  if ((ij_PID_var.error * ij_PID_var.ctrl) > 0 && abs(ij_PID_var.ctrl) >= outMax) { //Anti-Windup: Integral Clamping
    ij_PID_var.INT += 0; // IF the cmd is beyond the limits and has the same sign of the error, it stops updating the integral term
  }
  else {
    ij_PID_var.INT += ki * (ij_PID_var.error) * (ts); //ELSE compute the integral term as usual
  }
  ij_PID_var.error_old = ij_PID_var.error;  // Store the error for the next iteration
  ij_PID_var.ctrl = kp * ij_PID_var.error + ij_PID_var.INT + ij_PID_var.DER; // controller output, Parallel PID form
  ij_PID_var.ctrl = constrain(ij_PID_var.ctrl , outMin , outMax); // Controller output Saturation
  return ij_PID_var; //Return the structure with all these variables
}
