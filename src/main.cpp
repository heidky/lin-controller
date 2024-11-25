
#include <Arduino.h>
#include <math.h>
#include "AS5600.h"
#include <arduino-timer.h>

#include "motion/HBridge.h"
#include "motion/MotorController.h"
#include "motion/Motion.h"
#include "remote/BLRemote.h"
#include "remote/LVS.h"
// #include "remote/Config.h"

/** >> uncomment to wait for serial */
// #define DEV_MODE

/** >> uncomment to plot to serial */
#define DEV_PLOT

// =====> PIN CONFIG <=====
#define PIN_IN1 2
#define PIN_IN2 3

constexpr float DRUM_RADIUS_CM     = 1.5;
constexpr float ROD_MIN_CM         = 5.0;
constexpr float ROD_MAX_CM         = 50;
constexpr float ROD_END_MARGIN_CM  = 0.5;
constexpr float MAX_SPEED_CMS      = 40.0;
constexpr bool INVERT_POSITIVE_DIRECTION = true;

constexpr float CALIBRATION_SPEED_CMS  = 1.0;
constexpr float CALIBRATION_FORCE_PERC = 0.25;

constexpr int CONTROL_LOOP_MS = 10; // 100hz control loop
constexpr int WARN_SLOW_LOOP_MS = 12;


constexpr float cm_per_rev = 2.0 * PI * DRUM_RADIUS_CM;
constexpr float rev_per_cm = 1.0 / cm_per_rev;

constexpr float calibration_max_rev = ROD_MAX_CM * 1.25 * rev_per_cm;
constexpr float margin_rev = ROD_END_MARGIN_CM * rev_per_cm;


AS5600 encoder;   //  use default Wire
HBridge motor(PIN_IN2, PIN_IN1);
MotorController controller(encoder, motor, INVERT_POSITIVE_DIRECTION);
Motion motion(controller, cm_per_rev);
BLRemote remote(controller, motion);
LVS lvs;
// Configurator configurator(controller);

Timer<10, micros> timer;

enum class State
{
  Boot = 0,
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
void Error_update();

void checkSlowLoopTime();
void checkMotorDirectionError();

bool timed_loop(void *);



State state = State::Boot;
bool calibrated = false;
float max_position = 0;
float min_position = 0;
float real_rod_length = 0;

long last_loop_millis = -1;
int direction_error_counter = 0;
long loop_count = 0;


void setState(State s)
{
  state = s;
  int state_code = static_cast<int>(s);
  auto info = remote.getInfoRef();
  info->state_code = state_code;
  remote.notifyInfoUpdate();
}


void setup()
{
  Serial.begin(57600);
#ifdef DEV_MODE
  while(!Serial);
#endif
  Serial.println("\n\n**LIN Controller [v0.0]**\n");

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

  // Motor controller check
  controller.begin();
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

  timer.every(CONTROL_LOOP_MS * 1000, timed_loop);
}


void loop() 
{
  timer.tick();
}


bool timed_loop(void *) 
{
  BLE.poll();

  if(state == State::CalibrationZero)
    CalibrationZero_update();
  else if(state == State::CalibrationMax)
    CalibrationMax_update();
  else if(state == State::Operative)
    Operative_update();
  else if(state == State::Error) {
    Error_update();
  }

  controller.update();

  checkSlowLoopTime();

#ifdef DEV_PLOT
  if (loop_count % 10 == 0) {
    Serial.print("pos_curr:");
    Serial.print(controller.getCurrentPosition());
    Serial.print(",pos_tar:");
    Serial.print(controller.getTargetPosition());
    Serial.print(",speed_cur:");
    Serial.print(controller.getCurrentSpeed());
    Serial.print(",speed_tar:");
    Serial.print(controller.getTargetSpeed());
    Serial.print(",th:");
    Serial.print(controller.getThrottleOutput());
    Serial.println("");
  }
#endif

  loop_count += 1;
  return true;
}


// ============== CALIBRATION ================
void CalibrationZero_enter()
{
  setState(State::CalibrationZero);
  controller.setForce(CALIBRATION_FORCE_PERC);
  controller.setSpeed(CALIBRATION_SPEED_CMS);
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
  setState(State::CalibrationMax);
  controller.setForce(CALIBRATION_FORCE_PERC);
  controller.setSpeed(CALIBRATION_SPEED_CMS);
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
    real_rod_length = end_travel * cm_per_rev;
    motion.setPostionLimits(min_position, max_position);

    float adjusted_rod_length = (max_position - min_position) * cm_per_rev;
    RemoteInfo *info = remote.getInfoRef();
    info->max_speed = MAX_SPEED_CMS;
    info->rod_lenght = adjusted_rod_length;
    remote.notifyInfoUpdate();

    if(real_rod_length > ROD_MIN_CM && real_rod_length < ROD_MAX_CM)
    {
      Serial.println("CALIBRATION: Max Set");
      Serial.print("real rod lenght: ");
      Serial.print(real_rod_length);
      Serial.println("cm");

      Serial.print("adjusted rod lenght: ");
      Serial.print(adjusted_rod_length);
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
  setState(State::Operative);
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
      // motion.moveToPerc(value);
      motion.doVibe(.5, value);
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
  setState(State::Error);
  controller.off();
  Serial.println(msg);

  RemoteInfo *info = remote.getInfoRef();
  info->error_code = code;
  remote.notifyInfoUpdate();
}

void Error_update() {
  controller.off();
  remote.checkResetRequest();
}


void checkSlowLoopTime()
{
  const long now = millis();
  const int delta = now - last_loop_millis;
  if(last_loop_millis > 0 && now - last_loop_millis > WARN_SLOW_LOOP_MS)
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
