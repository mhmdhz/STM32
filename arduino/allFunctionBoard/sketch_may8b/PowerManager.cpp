#include "PowerManager.h"

PowerManager::PowerManager(uint8_t powerSwitchPin, uint8_t ledPin, uint8_t led2Pin, uint8_t buzzerPin, uint8_t powerKeyPin)
  : powerSwitchPin(powerSwitchPin), ledPin(ledPin), led2Pin(led2Pin), buzzerPin(buzzerPin), powerKeyPin(powerKeyPin) {}

void PowerManager::begin() {
  pinMode(powerSwitchPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(led2Pin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(powerKeyPin, OUTPUT);
  
  digitalWrite(powerSwitchPin, LOW);
  digitalWrite(ledPin, LOW);
  digitalWrite(led2Pin, HIGH);
  digitalWrite(buzzerPin, LOW);
  digitalWrite(powerKeyPin, LOW);
}

void PowerManager::turnOnPower() {
  digitalWrite(powerSwitchPin, HIGH);
}

void PowerManager::turnOffPower() {
  digitalWrite(powerSwitchPin, LOW);
}

float PowerManager::readVoltage(uint8_t analogPin, float referenceVoltage) {
  int sensorValue = analogRead(analogPin);
  float voltage = sensorValue * (referenceVoltage / 1023.0);
  return voltage;
}
