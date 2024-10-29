#ifndef MQTT_HANDLER_H
#define MQTT_HANDLER_H

#include <PubSubClient.h>
#include <ArduinoJson.h>

class MQTTHandler {
public:
  MQTTHandler(PubSubClient& client);
  void parseAndPublishData(const char* message, float rpm, float speed, float latitude, float longitude, bool obdpower, const String& dateTime, const String& batteryStatus);
  void publishData(float rpm, float speed, float latitude, float longitude, float voltage, bool obdpower, const String& dateTime, const String& batteryStatus);
  void publishLatLong(float latitude, float longitude);
  void handleMessage(char* topic, byte* payload, unsigned int len, bool& PowerSwitchState, uint8_t Led2Pin, HardwareSerial& SERIAL_PORT);
  boolean connect(const char* clientId, const char* username, const char* password);
  
private:
  PubSubClient& mqttClient;
  const char* topicOut = "v1/devices/me/telemetry";
  const char* topicIn = "v1/devices/me/attributes";
};

#endif // MQTT_HANDLER_H
