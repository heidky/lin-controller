
#include <Arduino.h>
#include <math.h>
#include "AS5600.h"

#include "motion/HBridge.h"
#include "motion/MotorController.h"
#include "motion/Motion.h"
#include "remote/BLRemote.h"
#include "remote/LVS.h"
// #include "remote/Config.h"


#define DRUM_RADIUS_CM    1
#define ROD_MIN_CM        5
#define ROD_MAX_CM        50
#define MARGIN_CM         0.25
#define MAX_SPEED         15

#define CALIBRATION_SPEED 1
#define CALIBRATION_FORCE 0.4

#define SAFE_LOOP_MS 6
// #define DEV_PLOT
// #define DEV_MODE


const float cm_per_rev = 2 * PI * DRUM_RADIUS_CM;
const float rev_per_cm = 1 / cm_per_rev;

const float calibration_max_rev = ROD_MAX_CM * 1.25 * rev_per_cm;
const float margin_rev = MARGIN_CM * rev_per_cm;


AS5600 encoder;   //  use default Wire
HBridge motor(2, 3);
MotorController controller(encoder, motor);
Motion motion(controller, cm_per_rev);
BLRemote remote(controller);
LVS lvs;
// Configurator configurator(controller);


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

void Error_enter(const String &msg, int=1);
void checkSlowLoopTime();
void checkMotorDirectionError();



State state = State::Boot;
bool calibrated = false;
float max_position = 0;
float min_position = 0;
float rod_length = 0;

long last_loop_millis = -1;
int direction_error_counter = 0;



void setup()
{
  Serial.begin(57600);
#ifdef DEV_MODE
  while(!Serial);
#endif
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
    Serial.print(" [Too Strong] ");
  if(encoder.magnetTooWeak())
    Serial.print(" [Too Weak] ");
  Serial.println();
  delay(1000);

  // configurator.begin();
  bool remote_success = remote.begin();
  if(!remote_success)
  {
    Serial.println("ERROR: starting Bluetooth® Low Energy module failed!");
    while(true) delay(1000);
  }
  else
    Serial.println("REMOTE: Bluetooth® Low Energy module operative!");

  lvs.begin(remote.getPrimaryService());
  delay(100);
  remote.start();

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

  controller.update();

  checkSlowLoopTime();

#ifdef DEV_PLOT
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
    motion.setPostionLimits(min_position, max_position);

    RemoteInfo *info = remote.getInfoRef();
    info->max_speed = MAX_SPEED;
    info->rod_lenght = (max_position - min_position) * cm_per_rev;
    remote.notifyInfoUpdate();

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
  controller.setSpeed(3); 
  motion.moveToCm(0);

  Serial.println("OPERATIVE");
}


void Operative_update()   
{
  remote.update();

  if(lvs.available())
  {
    LVSCommand cmd = lvs.getCommand();
    float value = lvs.getValue();

    if(cmd == LVSCommand::Thrust || cmd == LVSCommand::Vibe)
    {
      motion.moveToPerc(value);
    }
    else if(cmd == LVSCommand::Vibe1)
    {
      motion.vibe_value = value;
    }
    else if(cmd == LVSCommand::Vibe2)
    {
      motion.vibe_throw = value;
    }

    // Serial.print(motion.vibe_throw);
    // Serial.print(" ");
    // Serial.println(motion.vibe_value);
  }

  motion.update();
}


// ============= ERROR ===============
void Error_enter(const String &msg, int code)
{
  state = State::Error;
  controller.off();
  Serial.println(msg);

  RemoteInfo *info = remote.getInfoRef();
  info->error_code = code;
  remote.notifyInfoUpdate();
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

    RemoteInfo *info = remote.getInfoRef();
    info->slow_loops_count += 1;
    remote.notifyInfoUpdate();
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
