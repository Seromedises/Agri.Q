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
