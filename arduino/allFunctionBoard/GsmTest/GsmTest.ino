#define TINY_GSM_MODEM_SIM808
#include "Arduino.h"
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#define SerialAT Serial2
#define SERIAL_PORT Serial5

HardwareSerial Serial2(PA_3, PA_2);   // Connected to SIM808 TX2,RX2
HardwareSerial Serial5(PD_2, PC_12);  // Connected to TTL to USB for serial debugging TX3,RX3

#define SIM808_POWERKEY PC10
String dateTime = "";
String batteryStatus = "";
float rpm = 1000;

#define Led PE5
#define Led2 PC2

const char apn[] = "mcinet";
const char user[] = "";
const char pass[] = "";
const char* broker = "thingsboard.cloud";
const char* topicOut = "v1/devices/me/telemetry";
const char* topicIn = "v1/devices/me/attributes";

TinyGsm modem(SerialAT);
TinyGsmClient client(modem);
PubSubClient mqtt(client);

bool mqttConnect() {
  if (!mqtt.connect("test-system1", "1oxlc4oogqobuv0vvskv", "")) {
    SERIAL_PORT.print("Failed MQTT connection, rc=");
    SERIAL_PORT.print(mqtt.state());
    SERIAL_PORT.println(" try again in 10 miliseconds");
    delay(10);
    return false;
  }
  SERIAL_PORT.println("Connected to MQTT Broker.");
  mqtt.subscribe(topicIn);
  return mqtt.connected();
}

void mqttCallback(char* topic, byte* payload, unsigned int len) {
  SERIAL_PORT.print("Message received on topic ");
  SERIAL_PORT.print(topic);
  SERIAL_PORT.print(": ");
  SERIAL_PORT.write(payload, len);
  SERIAL_PORT.println();

  // Parse the payload as a JSON object
  DynamicJsonDocument doc(512);
  DeserializationError error = deserializeJson(doc, payload, len);
  if (error) {
    SERIAL_PORT.print("Failed to parse MQTT message: ");
    SERIAL_PORT.println(error.c_str());
    return;
  }

  // Handle "key" in JSON
  if (doc.containsKey("key")) {
    bool keyState = doc["key"];

    if (keyState) {
      digitalWrite(Led2, LOW);
      SERIAL_PORT.println("RaspberryPi is On");
    } else {
      digitalWrite(Led2, HIGH);
      SERIAL_PORT.println("RaspberryPi is Off");
    }
  } else {
    SERIAL_PORT.println("Error: 'key' not found in the JSON payload");
  }

  // Handle "rpm" in JSON
  if (doc.containsKey("rpm")) {
    rpm = doc["rpm"];
    SERIAL_PORT.println(rpm);
  }
}

bool retryFunction(bool (*func)(), int maxAttempts, unsigned long maxTotalTime) {
  int attempts = 0;
  unsigned long startTime = millis();

  while (attempts < maxAttempts && (millis() - startTime) < maxTotalTime) {
    if (func()) {
      return true;  // Success
    }
    attempts++;
    SERIAL_PORT.print("Attempt ");
    SERIAL_PORT.print(attempts);
    SERIAL_PORT.println(" failed. Retrying...");

    // Optional: Wait before retrying
    delay(1000);  // 1 second delay between attempts
  }

  return false;  // All attempts failed or timeout exceeded
}

bool tryNetworkConnection() {
  if (modem.waitForNetwork()) {
    SERIAL_PORT.println("Network connected .");

    return true;  // Success
  }
  SERIAL_PORT.println("Network connection failed.");
  return false;  // Failure
}

bool tryGPRSConnection() {
  if (modem.gprsConnect(apn, user, pass)) {
    SERIAL_PORT.println("GPRS connected .");

    return true;  // Success
  }
  SERIAL_PORT.println("GPRS connection failed.");
  return false;  // Failure
}

bool tryMQTTConnection() {
  return mqttConnect();
}

bool gsmInitialize() {

  SERIAL_PORT.println("Modem: " + modem.getModemInfo());
  SERIAL_PORT.println("Searching for telco provider.");

  // Try to connect to the network
  if (!retryFunction(tryNetworkConnection, 3, 10000)) {
    // Return false if network connection fails
    return false;
  }
  SERIAL_PORT.println("Connected to telco.");

  SERIAL_PORT.println("Enabling battery charging...");
  modem.sendAT("+ECHARGE=1");
  String response = modem.stream.readStringUntil('\n');
  SERIAL_PORT.println(response);

  // Check Battery Status
  SERIAL_PORT.println("Checking battery status...");
  modem.sendAT("+CBC");  // Get battery status
  String batteryStatusResponse = modem.stream.readStringUntil('\n');
  SERIAL_PORT.println(batteryStatusResponse);

  // Signal Quality and GPRS Connection
  SERIAL_PORT.println("Signal Quality: " + String(modem.getSignalQuality()));
  SERIAL_PORT.println("Connecting to GPRS network.");

  // Try to connect to GPRS
  if (!retryFunction(tryGPRSConnection, 3, 10000)) {
    // Return false if GPRS connection fails
    return false;
  }
  SERIAL_PORT.println("Connected to GPRS: " + String(apn));

  mqtt.setServer(broker, 1883);
  mqtt.setCallback(mqttCallback);

  // Try to connect to the MQTT Broker
  if (!retryFunction(tryMQTTConnection, 3, 10000)) {
    // Return false if MQTT connection fails
    return false;
  }
  SERIAL_PORT.println("Connected to MQTT Broker: " + String(broker));

  // If all connections succeed, return true
  return true;
}


bool checkGsmConection(){

    if (!retryFunction(tryNetworkConnection, 1, 10000)) {
    // Return false if network connection fails
    return false;
  }

  if (!retryFunction(tryGPRSConnection, 1, 10000)) {
    // Return false if GPRS connection fails
    return false;
  }

    if (!retryFunction(tryMQTTConnection, 1, 10000)) {
    // Return false if MQTT connection fails
    return false;
  }

  return true ; 
}
void setup() {
  delay(100);
  pinMode(Led, OUTPUT);
  pinMode(Led2, OUTPUT);
  SERIAL_PORT.begin(115200);  // Set baud rate for OBD communication
  delay(100);
  SERIAL_PORT.println();
  SERIAL_PORT.println("*************** Initialize The GSM Started ***************");
  SerialAT.begin(115200);
  pinMode(SIM808_POWERKEY, OUTPUT);
  digitalWrite(SIM808_POWERKEY, LOW);
  delay(100);

  gsmInitialize();
  delay(100);
  SERIAL_PORT.println("GPS Turning ON ...");
  modem.enableGPS();
  delay(500);
  SERIAL_PORT.println("GPS Activated.");
}

void loop() {
  mqtt.loop();
 if (checkGsmConection()) {
    SERIAL_PORT.println("GSM is conected ");
  } else {
    SERIAL_PORT.println("GSM is not  conected");
  }
  delay(1000);
}
