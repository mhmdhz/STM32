#include "OBDHandler.h"

OBDHandler::OBDHandler(HardwareSerial& obdSerial, HardwareSerial& debugSerial)
  : obdSerial(obdSerial), debugSerial(debugSerial) {}

bool OBDHandler::begin() {
  obdSerial.begin(38400);
  if (!elm.begin(obdSerial)) {
    debugSerial.println("Failed to initialize ELM327");
    return false;
  }
  debugSerial.println("ELM327 initialized successfully");
  return true;
}

float OBDHandler::getRPM() {
  float rpm = elm.rpm();
  return rpm;
}

float OBDHandler::getSpeed() {
  float speed = elm.kph();
  return speed;
}
