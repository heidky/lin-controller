#pragma once

#include <Arduino.h>
#include <ArduinoBLE.h>

#define LVS_SERVICE_UUID              "46300001-0023-4bd4-bbd5-a6920e4c5653"
#define LVS_TX_CHARASTERIC_UUID       "46300002-0023-4bd4-bbd5-a6920e4c5653"
#define LVS_RX_CHARASTERIC_UUID       "46300003-0023-4bd4-bbd5-a6920e4c5653"
#define LVS_IDENTIFIER                "F"
#define LVS_ADDRESS                   "0082059AD3BD"
#define LVS_FIRMWARE                  "11"

#define RX_TX_BYTE_SIZE 300

enum class LVSCommand
{
  None = 0,
  Vibe,
  Thrust,
};


class LVS
{
public:
  const String local_name = String("LVS-") + LVS_IDENTIFIER + "0" + LVS_FIRMWARE;
  const String device_type = String(LVS_IDENTIFIER) + ":" + LVS_FIRMWARE + ":" + LVS_ADDRESS + ";";

  LVS() {};

  bool begin();
  void end();
  void start();
  void stop();
  bool available();

  LVSCommand getCommand() const { return command; }
  int getRawValue() const { return raw_value; }
  float getValue() const { return value; }

protected:
  // service to be connect to
  BLEService service = BLEService(LVS_SERVICE_UUID); // create service
  // create switch characteristic and allow remote device to read and write
  BLEStringCharacteristic tx = BLEStringCharacteristic(LVS_TX_CHARASTERIC_UUID, BLERead | BLEWrite | BLEWriteWithoutResponse, RX_TX_BYTE_SIZE);
  // create button characteristic and allow remote device to get notifications
  BLEStringCharacteristic rx = BLEStringCharacteristic(LVS_RX_CHARASTERIC_UUID, BLERead | BLENotify, RX_TX_BYTE_SIZE);

  LVSCommand command = LVSCommand::None;
  int raw_value = 0;
  float value = 0.0;

  bool handleTx(const String &data);
};