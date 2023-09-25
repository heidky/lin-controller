#include "AS5600.h"
#include "MotorController.h"
#include <math.h>

#define STUCK_UPDATE_MS 500
#define STUCK_MARGIN 30


//dynamic speed tuning: Kp=1/speed  alpha=speed/100
// MotorController::MotorController(AS5600 &encoder)
// {
// }

void MotorController::begin()
{
  motor.begin();

  encoder.begin();  //  set direction pin.
  encoder.setDirection(AS5600_CLOCK_WISE);  // default, just be explicit.
  encoder.setHysteresis(0);
  encoder.resetCumulativePosition();

  position_pid.SetOutputLimits(-1.0, 1.0);
  position_pid.SetMode(AUTOMATIC);
  position_pid.SetSampleTime(1);

  speed_pid.SetOutputLimits(-1.0, 1.0);
  speed_pid.SetMode(AUTOMATIC);
  speed_pid.SetSampleTime(1);
}

void MotorController::update()
{
  current_position = encoder.getCumulativePosition() * AS5600_RAW_TO_DEGREES / 360.0;
  current_speed = encoder.getAngularSpeed() / 360.0;

  position_pid.Compute();
  speed_pid.Compute();

  if(!isAligned())
  {
    updateMotor(throttle_output);
    if(!isTraveling()) travelStart();
  }
  else
  {
    updateMotor(0);
    if(isTraveling()) travelEnd();
  }

  checkStuck();
}


void MotorController::off()
{
  cancelMove();
  motor.disable();
}


void MotorController::setSpeed(float s)
{
  if(s < 0 || s == speed) return;

  this->speed = s;
  position_pid.SetOutputLimits(-s, +s);
}


void MotorController::setForce(float f)
{
  if(f < 0 || f > 1) return;

  this->force = f;
}


void MotorController::moveTo(float pos)
{
  this->target_position = pos;
}


void MotorController::cancelMove()
{
  this->target_position = current_position;
  travelEnd();
}


void MotorController::resetPosition(float pos)
{
  const int raw_pos = round(pos * 360.0 / AS5600_DEGREES_TO_RAW);
  encoder.resetCumulativePosition(raw_pos);
  current_position = pos;
  cancelMove();
}


bool MotorController::isAligned() {
  return abs(current_position - target_position) < tolerance;
}


bool MotorController::isStuck()
{
  if (!isTraveling()) return false;
  long delta_time = millis() - travel_start_time;
  if(delta_time < STUCK_UPDATE_MS * 1.5) return false;

  return abs(last_pos_1 - last_pos_2) * STUCK_MARGIN < speed * (STUCK_UPDATE_MS/1000.0);
}

bool MotorController::checkDirectionError()
{
  if (!isAligned() && isTraveling() && millis() - travel_start_time > 250 && abs(current_speed) > 0.5*speed)
    return (target_position - current_position) * current_speed < 0;

  return false;
}


void MotorController::setTuningsV(double p, double i, double d)
{
  if(p < 0) p = speed_pid.GetKp();
  if(i < 0) i = speed_pid.GetKi();
  if(d < 0) d = speed_pid.GetKd();

  speed_pid.SetTunings(p, i, d);
}

void MotorController::setTuningsP(double p, double i, double d)
{
  if(p < 0) p = position_pid.GetKp();
  if(i < 0) i = position_pid.GetKi();
  if(d < 0) d = position_pid.GetKd();

  position_pid.SetTunings(p, i, d);
}

void MotorController::updateMotor(float t)
{
  if(isAligned()) 
  {
    motor.setThrottle(0);
    last_throttle_output = 0;
    return;
  }

  double smoothed_throttle = throttle_smooth_alpha*t + (1.0-throttle_smooth_alpha)*last_throttle_output;
  last_throttle_output = smoothed_throttle;
  t = smoothed_throttle;

  // clamp to max allowed force
  if (t > force) t = force;
  else if(t < -force) t = -force;

  motor.setThrottle(t);
}

void MotorController::travelStart()
{
  if(in_travel) return;

  travel_start_time = millis();
  travel_start_pos = current_position;
  in_travel = true;
}

void MotorController::travelEnd()
{
  if(!in_travel) return;
  
  travel_start_time = -1;
  travel_start_pos = current_position;
  in_travel = false;
}


void MotorController::checkStuck()
{
  long now = millis();
  if(now - last_pos_update > STUCK_UPDATE_MS)
  {
    last_pos_2 = last_pos_1;
    last_pos_1 = current_position;
    last_pos_update = now;
  }
}


