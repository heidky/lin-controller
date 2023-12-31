#include "Motion.h"
#include "math.h"

// #define VIBE_EASE_PERC 0.25
#define VIBE_RETRACT_SPEED 3.0
#define VIBE_MIN_SPEED 1.0
#define VIBE_MAX_SPEED 10.0


float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


float ease(float x, float x_margin, float y_min)
{
  if(x_margin <= 0.000001) return x;

  float p = 1.0;

  if(x < x_margin)
    p = x / x_margin;
  else if((1 - x) < x_margin)
    p = (1 - x) / x_margin;

  return p * (1 - y_min) + y_min;
}


void Motion::update()
{
  const float vibe_ref = vibe_zero;
  const float vibe_max = vibe_throw;

  if(vibe_value > 0.05 && vibe_throw > 0.05)
  { 
    is_vibing = true;

    if(!controller.isTraveling() || controller.isStuck())
    {
      if(vibe_to_ref)
      {
        moveToPerc(vibe_ref);
        vibe_to_ref = false;

        // Serial.print("to ref ");
        // Serial.println(getPerc());
      }
      else
      {
        moveToPerc(vibe_max);
        vibe_to_ref = true;

        // Serial.print("to max ");
        // Serial.println(getPerc());
      }
    }

    // const float ease_min = mapf(vibe_value, 0, 1, 1, .2);
    const float speed = mapf(vibe_value, 0, 1, VIBE_MIN_SPEED, VIBE_MAX_SPEED);
    const float e = ease(controller.travelPerc(), vibe_ease, .2);
    controller.setSpeed(speed * e);

    // const float smooth = mapf(vibe_value, 0, 1, 0.05, 0.9);
    float smooth;
    if (vibe_value > 0.75) smooth = 1.0/3;
    else if(vibe_value > 0.5) smooth = 1.0/6;
    else if(vibe_value > 0.25) smooth = 1.0/12;
    else smooth = 1.0/24;
    controller.setThrottleSmoothing(smooth);

    // Serial.print(controller.travelPerc());
    // Serial.print(" ");
    // Serial.println(e);
  }
  else if(is_vibing || prev_throw != vibe_throw)
  {
    controller.setSpeed(VIBE_RETRACT_SPEED);
    moveToPerc(vibe_max);
    is_vibing = false;
    prev_throw = vibe_throw;
  }
}

void Motion::moveToPerc(float perc)
{
  if(perc > 1) perc = 1.0;
  if(perc < 0) perc = 0.0;

  controller.moveTo((max_position - min_position) * perc + min_position);
}


void Motion::moveToCm(float cm)
{
  const float travel_lenght = max_position - min_position;
  if(cm < 0) cm = 0;
  if(cm > travel_lenght) cm = travel_lenght;

  moveToPerc(cm / travel_lenght);
}


void Motion::setVibeEase(float ease)
{
  if(ease < 0) ease = 0;
  else if(ease > 0.5) ease = 0.5;

  vibe_ease = ease;
}
