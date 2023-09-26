#include "Motion.h"
#include "math.h"

#define VIBE_EASE_PERC 0.25
#define VIBE_MIN_SPEED 3
#define VIBE_MAX_SPEED 15

void Motion::update()
{
  if(vibe_value > 0.1 && vibe_throw > 0.1)
  {
    float travel_perc = getPerc() / vibe_throw;

    if(!controller.isTraveling())
    {
      if(travel_perc > 0.5)
        moveToPerc(0);
      else
        moveToPerc(vibe_throw);
    }

    const float p = travel_perc > 0.5 ? 1 - travel_perc : travel_perc;

    float s = 1.0;
    if(p < VIBE_EASE_PERC) s = p/VIBE_EASE_PERC;

    const float speed = (VIBE_MIN_SPEED + (VIBE_MAX_SPEED - VIBE_MIN_SPEED) * s) * vibe_value;
    controller.setSpeed(speed);
  }
  else
  {
    controller.setSpeed((VIBE_MAX_SPEED - VIBE_MIN_SPEED) / 2);
    moveToPerc(0);
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