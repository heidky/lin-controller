#include <Arduino.h>
#include "MotorController.h"


class Motion
{
protected:
  MotorController &controller;
  float min_position;
  float max_position;

public:
  Motion(MotorController &controller, float cm_per_rev): controller(controller), cm_per_rev(cm_per_rev) {};

  const float cm_per_rev;

  void setPostionLimits(float min, float max) {
    max_position = max;
    min_position = min;
  }

  float vibe_throw = 0;
  float vibe_value = 0;
  bool vibe_to_ref = false;

  void update();

  void moveToPerc(float perc);
  void moveToCm(float cm);
  float getPerc() { return (controller.getCurrentPosition() - min_position) / (max_position - min_position); }
};