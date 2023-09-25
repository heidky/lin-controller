#include "LVS.h"

bool LVS::begin()
{
    // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");
    return false;
  }

  // set the local name peripheral advertises
  BLE.setLocalName(local_name.c_str());
  // set the UUID for the service this peripheral advertises:
  BLE.setAdvertisedService(service);

  // add the characteristics to the service
  service.addCharacteristic(rx);
  service.addCharacteristic(tx);

  // add the service
  BLE.addService(service);

  return true;
}


void LVS::end()
{
  BLE.end();
}


void LVS::start()
{
  rx.writeValue("");
  tx.writeValue("");

  // start advertising
  BLE.advertise();
}


void LVS::stop()
{
  rx.writeValue("");
  tx.writeValue("");

  // stop advertising
  BLE.stopAdvertise();
}


bool LVS::available()
{
  command = LVSCommand::None;
  raw_value = 0;
  value = 0.0;

  BLE.poll();

  if (tx.written()) {
    String data = tx.value();
    return handleTx(data);
  }
  else
    return false;
}


bool LVS::handleTx(const String &data)
{
  bool has_command = false;

  if(data.startsWith("Vibrate:"))
  {
    rx.writeValue("OK;");
    const int v = data.substring(8, data.length()-1).toInt();

    command = LVSCommand::Vibe;
    raw_value = v;
    value = v / 20.0;

    has_command = true;
  }
  else if(data.startsWith("Thrusting:"))
  {
    rx.writeValue("OK;");
    const int v = data.substring(10, data.length()-1).toInt();

    command = LVSCommand::Thrust;
    raw_value = v;
    value = v / 20.0;

    has_command = true;
  }
  else if(data == "DeviceType;")
  {
    rx.writeValue(device_type);
  }
  else if(data == "Battery;")
  {
    rx.writeValue("100;");
  }

  return has_command;
}
