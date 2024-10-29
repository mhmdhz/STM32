#ifndef POWER_MANAGER_H
#define POWER_MANAGER_H

#include "Arduino.h"

class PowerManager {
public:
  PowerManager(uint8_t powerSwitchPin, uint8_t ledPin, uint8_t led2Pin, uint8_t buzzerPin, uint8_t powerKeyPin);
  void begin();
  void turnOnPower();
  void turnOffPower();
  float readVoltage(uint8_t analogPin, float referenceVoltage);

private:
  uint8_t powerSwitchPin;
  uint8_t ledPin;
  uint8_t led2Pin;
  uint8_t buzzerPin;
  uint8_t powerKeyPin;
};

#endif // POWER_MANAGER_H
