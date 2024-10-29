#include "MQTTHandler.h"

MQTTHandler::MQTTHandler(PubSubClient& client) : mqttClient(client) {}

// Parse incoming message and publish data
void MQTTHandler::parseAndPublishData(const char* message, float rpm, float speed, float latitude, float longitude, bool obdpower, const String& dateTime, const String& batteryStatus) {
  StaticJsonDocument<500> doc;
  DeserializationError error = deserializeJson(doc, message);
  if (error) {
    Serial.println("Failed to parse JSON in parseAndPublishData");
    return;
  }

  int sleepTime = doc["sleep_time"];
  int noFaceTime = doc["noface_time"];
  int distractionTime = doc["distraction_time"];
  int visible = doc["visible"];
  int tracked = doc["tracked"];
  unsigned long ts = doc["ts"];

  StaticJsonDocument<500> publishDoc;
  publishDoc["RPM"] = rpm;
  publishDoc["speed"] = speed;
  publishDoc["sleep_time"] = sleepTime;
  publishDoc["noface_time"] = noFaceTime;
  publishDoc["distraction_time"] = distractionTime;
  publishDoc["latitude"] = latitude;
  publishDoc["longitude"] = longitude;
  publishDoc["visible"] = visible;
  publishDoc["tracked"] = tracked;
  publishDoc["ts"] = ts;
  publishDoc["OBD2_Power_connection"] = obdpower;
  publishDoc["Time"] = dateTime;
  publishDoc["BatteryStatus"] = batteryStatus;

  char buffer[500];
  serializeJson(publishDoc, buffer);
  mqttClient.publish(topicOut, buffer);

  if (mqttClient.connected()) {
    mqttClient.loop();
  }
}

// Publish basic telemetry data
void MQTTHandler::publishData(float rpm, float speed, float latitude, float longitude, float voltage, bool obdpower, const String& dateTime, const String& batteryStatus) {
  StaticJsonDocument<500> doc;
  doc["RPM"] = rpm;
  doc["speed"] = speed;
  doc["latitude"] = latitude;
  doc["longitude"] = longitude;
  doc["systemVoltage"] = voltage;
  doc["OBD2_Power_connection"] = obdpower;
  doc["Time"] = dateTime;
  doc["BatteryStatus"] = batteryStatus;

  char buffer[500];
  serializeJson(doc, buffer);
  mqttClient.publish(topicOut, buffer);

  if (mqttClient.connected()) {
    mqttClient.loop();
  }
}

// Publish only latitude and longitude
void MQTTHandler::publishLatLong(float latitude, float longitude) {
  StaticJsonDocument<200> doc;
  doc["latitude"] = latitude;
  doc["longitude"] = longitude;

  char buffer[200];
  serializeJson(doc, buffer);
  mqttClient.publish(topicOut, buffer);

  if (mqttClient.connected()) {
    mqttClient.loop();
  }
}

// Handle incoming MQTT messages
void MQTTHandler::handleMessage(char* topic, byte* payload, unsigned int len, bool& PowerSwitchState, uint8_t Led2Pin, HardwareSerial& SERIAL_PORT) {
  SERIAL_PORT.print("Message received:");
  SERIAL_PORT.write(payload, len);
  SERIAL_PORT.println();

  // Parse the payload as a JSON object
  DynamicJsonDocument doc(512);
  DeserializationError error = deserializeJson(doc, payload, len);
  if (error) {
    SERIAL_PORT.println("Failed to parse JSON in handleMessage");
    return;
  }

  // Handle "key"
  if (doc.containsKey("key")) {
    bool keyState = doc["key"];
    if (keyState) {
      PowerSwitchState = true;
      digitalWrite(Led2Pin, LOW);
      SERIAL_PORT.println("RaspberryPi is On");
    } else {
      PowerSwitchState = false;
      digitalWrite(Led2Pin, HIGH);
      SERIAL_PORT.println("RaspberryPi is Off");
    }
  } else {
    SERIAL_PORT.println("Error: 'key' not found in the JSON payload");
  }

  // Handle "rpm"
  if (doc.containsKey("rpm")) {
    int rpmValue = doc["rpm"];
    SERIAL_PORT.println(rpmValue);
  }
}

// Connect to MQTT Broker
boolean MQTTHandler::connect(const char* clientId, const char* username, const char* password) {
  if (!mqttClient.connect(clientId, username, password)) {
    Serial.print(".");
    return false;
  }
  Serial.println("Connected to MQTT broker.");
  mqttClient.subscribe(topicIn);
  return mqttClient.connected();
}
