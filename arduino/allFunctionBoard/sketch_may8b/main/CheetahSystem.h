#ifndef CHEETAH_SYSTEM_H
#define CHEETAH_SYSTEM_H
#define TINY_GSM_MODEM_SIM808

#include "Arduino.h"
#include "MQTTHandler.h"
#include "OBDHandler.h"
#include "PowerManager.h"
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "ELMduino.h"

class CheetahSystem {
public:
  CheetahSystem();
  void initialize();
  void run();

private:
  // Static callback to match PubSubClient's signature
  static void staticMqttCallback(char* topic, byte* payload, unsigned int len);
  void mqttCallback(char* topic, byte* payload, unsigned int len);
  boolean mqttConnect();
  
  // Serial Ports
  HardwareSerial OBD2_PORT;
  HardwareSerial SerialAT;
  HardwareSerial Raspberrypi;
  HardwareSerial SERIAL_PORT;

  // Modules
  MQTTHandler mqttHandler;
  OBDHandler obdHandler;
  PowerManager powerManager;

  // GSM and MQTT
  TinyGsm modem;
  TinyGsmClient gsmClient;
  PubSubClient mqttClient;

  // Configuration Constants
  const char* apn = "mcinet";
  const char* user = "";
  const char* pass = "";
  const char* broker = "ero-services.ir";
  const char* topicOut = "v1/devices/me/telemetry";
  const char* topicIn = "v1/devices/me/attributes";

  // Variables
  float latitude;
  float longitude;
  float rpm;
  float speed;
  float voltage;
  String dateTime;
  String batteryStatus;
  
  // Timing
  unsigned long lastPublishGpsTime;
  const unsigned long publishIntervalGps = 600000; // 10 minutes
  unsigned long lastPublishrpmTime;
  const unsigned long publishIntervalrpm = 20000;  // 20 seconds

  // Other Variables
  String receivedMessage;
  bool processDataFromUART;
  bool PowerSwitchState;
  bool obdpower;

  // Constants
  const float referenceVoltage = 12.0;

  // Pin Definitions
  static const uint8_t analogInputPin = PA1;
  static const uint8_t PowerSwitchPin = PE3;
  static const uint8_t buzzerPin = PE9;
  static const uint8_t LedPin = PE5;
  static const uint8_t Led2Pin = PC2;
  static const uint8_t SIM808_POWERKEYPin = PC10;
  static const uint8_t FAN_PWMPin = PA6;
  static const uint8_t IR_PWMPin = PB14;

  // Static instance pointer for callback
  static CheetahSystem* instance;
};

#endif // CHEETAH_SYSTEM_H
