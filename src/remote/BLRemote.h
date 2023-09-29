#pragma once
#include <Arduino.h>
#include <ArduinoBLE.h>
#include "../motion/MotorController.h"
#include "../motion/Motion.h"


struct RemoteInfo
{
  int state_code = 0;
  int error_code = 0;
  int slow_loops_count = 0;
  float rod_lenght = -1;
  float max_speed = -1;
};


class BLRemote
{
protected:
  bool active = false;
  MotorController &controller;
  Motion &motion;
  RemoteInfo info_value;

  BLEService service = BLEService("50300001-0023-4bd4-bbd5-a6920e4c5653");

  BLEStringCharacteristic config = BLEStringCharacteristic("50300005-0023-4bd4-bbd5-a6920e4c5653", BLERead | BLENotify, 200);
  BLEStringCharacteristic config_rx = BLEStringCharacteristic("50300006-0023-4bd4-bbd5-a6920e4c5653", BLERead | BLEWrite, 20);

  BLEStringCharacteristic info = BLEStringCharacteristic("50300007-0023-4bd4-bbd5-a6920e4c5653", BLERead | BLENotify, 200);

  BLEStringCharacteristic motion_rx = BLEStringCharacteristic("50300008-0023-4bd4-bbd5-a6920e4c5653", BLERead | BLEWrite | BLEWriteWithoutResponse, 50);


  void updateConfig();
  void handleConfigRx(const String &data);

  void handleMotionRx(const String &data);

public:
  BLRemote(MotorController &controller, Motion &motion): controller(controller), motion(motion) {}

  bool begin();
  void start();
  void end();

  void update();
  bool hasBegun() const { return active; }
  BLEService &getPrimaryService() { return service; }

  RemoteInfo* getInfoRef() { return &info_value; }
  void notifyInfoUpdate();
};