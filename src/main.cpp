
#include <Arduino.h>
#include <math.h>
#include "AS5600.h"

#include "motion/HBridge.h"
#include "motion/MotorController.h"
#include "remote/LVS.h"
#include "remote/Config.h"


#define DRUM_RADIUS_CM    1
#define ROD_MIN_CM        5
#define ROD_MAX_CM        50
#define MARGIN_CM         0.25

#define CALIBRATION_SPEED 1
#define CALIBRATION_FORCE 0.4

#define SAFE_LOOP_MS 6
// #define PLOT_DEBUG


const float cm_per_rev = 2 * PI * DRUM_RADIUS_CM;
const float rev_per_cm = 1 / cm_per_rev;

const float calibration_max_rev = ROD_MAX_CM * 1.25 * rev_per_cm;
const float margin_rev = MARGIN_CM * rev_per_cm;


AS5600 encoder;   //  use default Wire
HBridge motor(2, 3);
MotorController controller(encoder, motor);
LVS lvs;
Configurator configurator(controller);


enum class State
{
  Boot,
  CalibrationZero,
  CalibrationMax,
  Operative,
  Error,
};


void CalibrationZero_enter();
void CalibrationZero_update();

void CalibrationMax_enter();
void CalibrationMax_update();

void Operative_enter();
void Operative_update();

void Error_enter(const String &msg);
void checkSlowLoopTime();
void checkMotorDirectionError();



State state = State::Boot;
bool calibrated = false;
float max_position = 0;
float min_position = 0;
float rod_length = 0;

long last_loop_millis = -1;
int direction_error_counter = 0;




void moveToPerc(float perc)
{
  if(perc > 1) perc = 1.0;
  if(perc < 0) perc = 0.0;

  controller.moveTo((max_position - min_position) * perc + min_position);
}


void moveToCm(float cm)
{
  const float travel_lenght = max_position - min_position;
  if(cm < 0) cm = 0;
  if(cm > travel_lenght) cm = travel_lenght;

  moveToPerc(cm / travel_lenght);
}


void setup()
{
  Serial.begin(57600);
  // while(!Serial);
  Serial.println("\n\n**LIN Controller [v0.0]**\n");

  // Motor controller check
  controller.begin();
  delay(1000);

  // Enecoder check
  encoder.begin();
  bool encoder_connected = encoder.isConnected();
  Serial.print("Encoder connected: ");
  Serial.println(encoder_connected);
  if(!encoder_connected) while(true) delay(1000);

  bool encoder_manget_ok = encoder.detectMagnet();
  Serial.print("Magnet Status: ");
  Serial.print(encoder_manget_ok);

  if(encoder.magnetTooStrong())
    Serial.print("[Too Strong] ");
  if(encoder.magnetTooWeak())
    Serial.print("[Too Weak] ");
  Serial.println();
  delay(1000);

  // configurator.begin();
  CalibrationZero_enter();
}


void loop() 
{
  if(state == State::CalibrationZero)
    CalibrationZero_update();
  else if(state == State::CalibrationMax)
    CalibrationMax_update();
  else if(state == State::Operative)
    Operative_update();
  else if(state == State::Error)
    controller.off();

  configurator.update();
  controller.update();

  checkSlowLoopTime();

#ifdef PLOT_DEBUG
  Serial.print(">");
  Serial.print(controller.getCurrentPosition());
  Serial.print(" ");
  Serial.print(controller.getTargetPosition());
  Serial.print(" ");
  Serial.print(controller.getCurrentSpeed());
  Serial.print(" ");
  Serial.print(controller.getTargetSpeed());
  Serial.print(" ");
  Serial.print(controller.getThrottleOutput());
  Serial.println("");
#endif
}


// ============== CALIBRATION ================
void CalibrationZero_enter()
{
  state = State::CalibrationZero;
  controller.setForce(CALIBRATION_FORCE);
  controller.setSpeed(CALIBRATION_SPEED);
  controller.moveTo(-calibration_max_rev);
  Serial.println("CALIBRATION: begin");
}


void CalibrationZero_update()
{
  if(controller.isStuck())
  {
    controller.cancelMove();
    controller.resetPosition(0);
    Serial.println("CALIBRATION: Zero Set!");
    CalibrationMax_enter();
  }
  else if(controller.isAligned())
    Error_enter("CALIBRATION ERROR: No rod detected during zero set");

  checkMotorDirectionError();
}


void CalibrationMax_enter()
{
  state = State::CalibrationMax;
  controller.setForce(CALIBRATION_FORCE);
  controller.setSpeed(CALIBRATION_SPEED);
  controller.moveTo(calibration_max_rev);
}


void CalibrationMax_update()
{
  if(controller.isStuck())
  {
    controller.cancelMove();

    const float end_travel = controller.getCurrentPosition();
    max_position = end_travel - margin_rev;
    min_position = margin_rev;
    rod_length = end_travel * cm_per_rev;

    if(rod_length > ROD_MIN_CM && rod_length < ROD_MAX_CM)
    {
      Serial.println("CALIBRATION: Max Set");
      Serial.print("Rod lenght: ");
      Serial.print(rod_length);
      Serial.println("cm");

      Operative_enter();
    }
    else
      Error_enter("CALIBRATION ERROR: Illegal rod size");
  }
  else if(controller.isAligned())
    Error_enter("CALIBRATION ERROR: No rod detected during max set");
  
  checkMotorDirectionError();
}

// ============ OPERATIVE ================
void Operative_enter()
{
  state = State::Operative;
  controller.setForce(1);
  controller.setSpeed(20); 
  moveToPerc(0);

  lvs.begin();
  lvs.start();
  Serial.println("OPERATIVE");
}


void Operative_update()   
{
  if(lvs.available())
  {
    LVSCommand cmd = lvs.getCommand();
    if(cmd == LVSCommand::Thrust || cmd == LVSCommand::Vibe)
    {
      float value = 1 - lvs.getValue();
      moveToPerc(value);
    }
  }

  if(!controller.isTraveling()) 
  {
    if (controller.getCurrentPosition() > 0.5)
      moveToPerc(0);
    else
      moveToPerc(0.2);
  }

  // if((millis()/1000) % 2 == 0)
  //   moveToPerc(0);
  // else
  //   moveToPerc(1);
}


// ============= ERROR ===============
void Error_enter(const String &msg)
{
  state = State::Error;
  controller.off();
  Serial.println(msg);
}


void checkSlowLoopTime()
{
  const long now = millis();
  const int delta = now - last_loop_millis;
  if(last_loop_millis > 0 && now - last_loop_millis > SAFE_LOOP_MS)
  {
    Serial.print("SLOW LOOP: ");
    Serial.print(delta);
    Serial.println("ms");
  }
  last_loop_millis = now;
}


void checkMotorDirectionError()
{
  if(controller.checkDirectionError())
    direction_error_counter += 1;
  else
    direction_error_counter = 0;

  if(direction_error_counter > 10)
    Error_enter("ERROR: Wrong motor orientation. Please invert motor wires.");
}
