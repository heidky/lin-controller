#pragma once
#include <Arduino.h>
#include <ArduinoBLE.h>
#include "MotorController.h"


class Configurator
{
protected:
  bool active = false;
  MotorController &controller;

  BLEService service = BLEService("46300001-0023-4bd4-bbd5-a6920e4c5653");
  BLEStringCharacteristic config = BLEStringCharacteristic("46300005-0023-4bd4-bbd5-a6920e4c5653", BLERead | BLENotify, 200);
  BLEStringCharacteristic config_rx = BLEStringCharacteristic("46300006-0023-4bd4-bbd5-a6920e4c5653", BLERead | BLEWrite, 20);

  void updateConfig();
  void handleRxWritten(const String &data);

public:
  Configurator(MotorController &controller): controller(controller) {}

  bool begin();
  void end();
  void update();
  bool hasBegun() const { return active; }
};