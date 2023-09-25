#pragma once
#include <Arduino.h>

#define HBRIDGE_MAX 254

class HBridge
{
private:
  pin_size_t pin_a, pin_b;
  float throttle = 0.0;
  bool enabled = true;

public:
  HBridge(pin_size_t a, pin_size_t b);
  void begin();
  void setThrottle(float value);
  void enable();
  void disable();
  bool isEnabled() const { return enabled; }
};