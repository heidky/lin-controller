#pragma once

#include "Arduino.h"
#include "HBridge.h"
#include "AS5600.h"
#include "PID_v1.h"


class MotorController
{
protected:
  double current_position = 0, target_position = 0;
  double current_speed = 0, target_speed = 0;
  double throttle_output = 0;
  double last_throttle_output = 0;

  float throttle_smooth_alpha = 0.05;
  float tolerance = 0.1;
  float force = 1.0;
  float speed = 1.0;

  AS5600 &encoder;
  HBridge &motor;

  PID position_pid, speed_pid;

  void updateMotor(float t);

  long last_pos_update = -1;
  float last_pos_1 = 0, last_pos_2 = 0;
  void checkStuck();

  bool in_travel = false;
  long travel_start_time = -1;
  float travel_start_pos = 0;

  void travelStart();
  void travelEnd();
  
public:
  MotorController(AS5600 &encoder, HBridge &motor):
    encoder(encoder),
    motor(motor),
    position_pid(&current_position, &target_speed, &target_position, 50.0, 0.0, 0.0, DIRECT),
    speed_pid(&current_speed, &throttle_output, &target_speed, 0.25, 0.0, 0.0, DIRECT)
  {};

  void begin();
  void update();

  void off();

  void setSpeed(float s);
  float getSpeed() { return speed; }

  void setForce(float f);
  float getForce() { return force; }

  void moveTo(float pos);
  // void moveTo(float pos, float speed);
  // void moveTo(float pos, float speed, float force);
  void cancelMove();

  bool isTraveling() const { return in_travel; }

  void resetPosition(float pos);
  bool isAligned();
  bool isStuck();
  bool checkDirectionError();

  float getCurrentPosition() const { return current_position; }
  float getTargetPosition() const { return target_position; }
  float getCurrentSpeed() const { return current_speed; }
  float getTargetSpeed() const { return target_speed; }
  float getThrottleOutput() const { return throttle_output; }

  double getVp() { return speed_pid.GetKp(); }
  double getVi() { return speed_pid.GetKi(); }
  double getVd() { return speed_pid.GetKd(); }

  double getPp() { return position_pid.GetKp(); }
  double getPi() { return position_pid.GetKi(); }
  double getPd() { return position_pid.GetKd(); }

  void setTuningsV(double p, double i, double d);
  void setTuningsP(double p, double i, double d);

  float getThrottleSmoothing() { return throttle_smooth_alpha; }
  void setThrottleSmoothing(float t) { if(t >=0) throttle_smooth_alpha = t; }
};