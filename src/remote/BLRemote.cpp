#include "BLRemote.h"


bool BLRemote::begin()
{
  // begin initialization
  if (!BLE.begin()) {
    return false;
  }

  // set the local name peripheral advertises
  // BLE.setLocalName("Hmlo");
  BLE.setLocalName("LVS-Z011");
  // set the UUID for the service this peripheral advertises:

  // add the characteristics to the service
  service.addCharacteristic(info);
  service.addCharacteristic(config);
  service.addCharacteristic(config_rx);
  service.addCharacteristic(motion_rx);

  // add the service


  // setup initial char values
  motion_rx.writeValue("");
  config_rx.writeValue("");
  updateConfig();

  info_value = RemoteInfo();
  notifyInfoUpdate();

  // start advertising
  return true;
}

void BLRemote::start()
{
  BLE.addService(service);
  BLE.setAdvertisedService(service);
  BLE.advertise();
}


void BLRemote::end()
{
  BLE.stopAdvertise();
  BLE.end();
}


void BLRemote::update()
{
  // BLE.poll();

  if(config_rx.written())
  {
    String data = config_rx.value();
    handleConfigRx(data);
  }

  if(motion_rx.written())
  {
    String data = motion_rx.value();
    handleMotionRx(data);
  }
}

void BLRemote::notifyInfoUpdate()
{
  char buffer[256];
  sprintf(buffer, "St:%d|Ec:%d|Rl:%.4f|Sl:%d|Ms:%.4f",
    info_value.state_code,
    info_value.error_code,
    info_value.rod_lenght,
    info_value.slow_loops_count,
    info_value.max_speed
  );
  info.writeValue(buffer);
}

void BLRemote::checkResetRequest()
{
  if(config_rx.written())
  {
    String data = config_rx.value();
    handleConfigRx(data);
  }
}

void BLRemote::updateConfig()
{
  char buffer[256];
  sprintf(buffer, "Vp:%.4f|Vi:%.4f|Vd:%.4f|Pp:%.4f|Pi:%.4f|Pd:%.4f|Ta:%.4f|F:%.4f|To:%.4f",
    controller.getVp(), controller.getVi(), controller.getVd(),
    controller.getPp(), controller.getPi(), controller.getPd(),
    controller.getThrottleSmoothing(),
    controller.getForce(), controller.getTolerance()
  );
  config.writeValue(buffer);
}


void BLRemote::handleConfigRx(const String &data)
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
  else if(param_name == "F")  controller.setForce(value);
  else if(param_name == "To") controller.setTolerance(value);
  else if(param_name == "Rst") {
    if(value > 0.5) {
      Serial.println("Reset");
      NVIC_SystemReset();
    }
  }

  updateConfig();
}


int extract_cmd(const String &data, int start_index, int data_len, char *out_indicator, float *out_float)
{
  *out_indicator = data[start_index];

  int cursor;
  for(cursor = start_index + 1; cursor < data_len; cursor++)
    if(data[cursor] == ' ') break;

  if(cursor - (start_index + 1) <= 0) return -1;
  *out_float = data.substring(start_index + 1, cursor).toFloat();

  return cursor + 1;
}

int extract_cmd_list(const String &data, char modes[], float values[], const int max_len)
{
  const int data_len = data.length();
  int cursor = 0;
  int count = 0;

  for (int i = 0; i < max_len && cursor < data_len; i++)
  {
    char mode = '0';
    float value = 0.0;

    cursor = extract_cmd(data, cursor, data_len, &mode, &value);
    if(cursor < 0) break;
    
    modes[count] = mode;
    values[count] = value;
    count ++;
  }

  return count;
}


void BLRemote::handleMotionRx(const String &data)
{ 
  char modes[10] = {};
  float values[10] = {};
  const int count = extract_cmd_list(data, modes, values, 10);

  if(count >= 2) 
  {
    if(modes[0] == 'p' and modes[1] == 's')
    {
      controller.setSpeed(values[1]);
      motion.moveToPerc(values[0]);

      motion.zeroVibe();
    }

    if(modes[0] == 't' and modes[1] == 's')
    {
      if(count >= 3 && modes[2] == 'e')
        motion.setVibeEase(values[2]);
        
      motion.doVibe(values[0], values[1]);
    }
  }
  else 
  {
    Serial.println("cmd invalid size");
  }
  
  // Serial.print(modes[0]);
  // Serial.print(modes[1]);
  // Serial.print(" ");
  // Serial.print(values[0]);
  // Serial.print(" ");
  // Serial.println(values[1]);
}


  // Serial.println("\nPARAMS:");
  // Serial.print("Vp: ");
  // Serial.print(controller.getVp());
  // Serial.print("| Vi: ");
  // Serial.print(controller.getVi());
  // Serial.print("| Vd: ");
  // Serial.println(controller.getVd());

  // Serial.print("Pp: ");
  // Serial.print(controller.getPp());
  // Serial.print("| Pi: ");
  // Serial.print(controller.getPi());
  // Serial.print("| Pd: ");
  // Serial.println(controller.getPd());
  
  // else if(param_name == "P")  controller.moveTo(value > 0 ? 1 : 0.2);
  // else if(param_name == "S")  controller.setSpeed(value);
  // else if(param_name == "F")  controller.setForce(value);





  // int cursor = 0;
  // int data_len = data.length();

  // char pos_mode;
  // float pos;
  // cursor = extract_cmd(data, cursor, data_len, &pos_mode, &pos);
  // if(cursor < 0 || cursor >= data_len) return;
  
  // char speed_mode;
  // float speed;
  // cursor = extract_cmd(data, cursor, data_len, &speed_mode, &speed);
  // if(cursor < 0) return;

  // if(pos_mode == 'p' and speed_mode == 's') {
  //   controller.setSpeed(speed);
  //   motion.moveToPerc(pos);

  //   motion.zeroVibe();
  // }

  // if(pos_mode == 't' and speed_mode == 's') {
  //   motion.doVibe(pos, speed);
  // }

  // Serial.print(pos_mode);
  // Serial.print(speed_mode);
  // Serial.print(" ");
  // Serial.print(pos);
  // Serial.print(" ");
  // Serial.println(speed);