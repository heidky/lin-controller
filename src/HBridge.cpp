#include "HBridge.h"


HBridge::HBridge(pin_size_t a, pin_size_t b)
{
  this->pin_a = a;
  this->pin_b = b;
}


void HBridge::begin()
{
  pinMode(pin_a, OUTPUT);
  pinMode(pin_b, OUTPUT);
}

void HBridge::enable()
{
  enabled = true;
}

void HBridge::disable()
{
  enabled = false;
}


void HBridge::setThrottle(float t)
{

  if(t > 1.0) t = 1.0;
  else if(t < -1.0) t = -1.0;

  const int pwm = round(abs(t) * HBRIDGE_MAX);

  if(t >= 0.0) 
  {
    analogWrite(pin_a, pwm);
    analogWrite(pin_b, 0);
  }
  else 
  {
    analogWrite(pin_a, 0);
    analogWrite(pin_b, pwm);
  }
}
