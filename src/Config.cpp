#include "Config.h"


bool Configurator::begin()
{
  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");
    return false;
  }

  // set the local name peripheral advertises
  BLE.setLocalName("Hmlo");
  // set the UUID for the service this peripheral advertises:
  BLE.setAdvertisedService(service);

  // add the characteristics to the service
  service.addCharacteristic(config);
  service.addCharacteristic(config_rx);

  // add the service
  BLE.addService(service);

  config_rx.writeValue("");
  updateConfig();

  // start advertising
  BLE.advertise();
  Serial.println("CONFIGURATOR: ready!");
  return true;
}


void Configurator::end()
{
  BLE.stopAdvertise();
  BLE.end();
}


void Configurator::update()
{
  BLE.poll();

  if(config_rx.written())
  {
    String data = config_rx.value();
    handleRxWritten(data);
  }
}


void Configurator::updateConfig()
{
  char buffer[200];
  sprintf(buffer, "Vp:%.4f|Vi:%.4f|Vd:%.4f|Pp:%.4f|Pi:%.4f|Pd:%.4f|Ta:%.4f|S:%.4f|F:%.4f",
    controller.getVp(), controller.getVi(), controller.getVd(),
    controller.getPp(), controller.getPi(), controller.getPd(),
    controller.getThrottleSmoothing(),
    controller.getSpeed(), controller.getForce()
  );
  config.writeValue(buffer);

  Serial.println("\nPARAMS:");
  Serial.print("Vp: ");
  Serial.print(controller.getVp());
  Serial.print("| Vi: ");
  Serial.print(controller.getVi());
  Serial.print("| Vd: ");
  Serial.println(controller.getVd());

  Serial.print("Pp: ");
  Serial.print(controller.getPp());
  Serial.print("| Pi: ");
  Serial.print(controller.getPi());
  Serial.print("| Pd: ");
  Serial.println(controller.getPd());
}


void Configurator::handleRxWritten(const String &data)
{
  int index = data.indexOf(":");
  if(index < 0) return;

  String param_name = data.substring(0, index);
  float value = data.substring(index+1, data.length()).toDouble();

  if(param_name == "Vp")      controller.setTuningsV(value, -1, -1);
  else if(param_name == "Vi") controller.setTuningsV(-1, value, -1);
  else if(param_name == "Vd") controller.setTuningsV(-1, -1, value);
  else if(param_name == "Pp") controller.setTuningsP(value, -1, -1);
  else if(param_name == "Pi") controller.setTuningsP(-1, value, -1);
  else if(param_name == "Pd") controller.setTuningsP(-1, -1, value);
  else if(param_name == "Ta") controller.setThrottleSmoothing(value);
  else if(param_name == "P")  controller.moveTo(value > 0 ? 1 : 0.2);
  else if(param_name == "S")  controller.setSpeed(value);
  else if(param_name == "F")  controller.setForce(value);

  updateConfig();
}
