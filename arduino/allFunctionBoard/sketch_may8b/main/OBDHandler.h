#ifndef OBD_HANDLER_H
#define OBD_HANDLER_H

#include <ELMduino.h>
#include <HardwareSerial.h>

class OBDHandler {
public:
  OBDHandler(HardwareSerial& obdSerial, HardwareSerial& debugSerial);
  bool begin();
  float getRPM();
  float getSpeed();

private:
  ELM327 elm;
  HardwareSerial& obdSerial;
  HardwareSerial& debugSerial;
};

#endif // OBD_HANDLER_H
