#define TINY_GSM_MODEM_SIM808
#include "Arduino.h"

#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "HardwareSerial.h"


HardwareSerial SerialAT(1);

// SoftwareSerial Serial1(2,3); // RX, TX

float latitude = 0.0;
float longitude = 0.0;
uint8_t chargeState = -99;
int8_t percent = -99;
uint16_t milliVolts = -9999;
//Network details
const char apn[] = "mcinet";
const char user[] = "";
const char pass[] = "";

// MQTT details
const char* broker = "ero-services.ir";
const char* topicOut = "v1/devices/me/telemetry";
const char* topicIn = "v1/devices/me/attributes";

TinyGsm modem(SerialAT);
TinyGsmClient client(modem);
PubSubClient mqtt(client);

void setup() {

  Serial.begin(9600);
  SerialAT.begin(115200, SERIAL_8N1, 16, 17);  // RX=16, TX=17
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN,LOW);


  // Serial.begin(9600);
  // Serial1.begin(9600);

  Serial.println("System start.");
  modem.restart();
  Serial.println("Modem: " + modem.getModemInfo());
  Serial.println("Searching for telco provider.");
  if (!modem.waitForNetwork()) {
    Serial.println("fail");
    while (true)
      ;
  }
  Serial.println("Connected to telco.");
  Serial.println("Signal Quality: " + String(modem.getSignalQuality()));

  Serial.println("Connecting to GPRS network.");
  if (!modem.gprsConnect(apn, user, pass)) {
    Serial.println("fail");
    while (true)
      ;
  }
  Serial.println("Connected to GPRS: " + String(apn));

  mqtt.setServer(broker, 1883);
  mqtt.setCallback(mqttCallback);
  Serial.println("Connecting to MQTT Broker: " + String(broker));
  while (mqttConnect() == false) continue;

  Serial.println();
  modem.enableGPS();
}

void loop() {
    digitalWrite(LED_BUILTIN,HIGH);

  String gps_raw = modem.getGPSraw();
  delay(100);
  Serial.println(gps_raw);  // Print latitude with 6 decimal places
  modem.getBattStats(chargeState, percent, milliVolts);
  delay(100);

  String dateTime = modem.getGSMDateTime(DATE_TIME);
  delay(100);

  modem.getGPS(&latitude, &longitude);
  delay(100);
      digitalWrite(LED_BUILTIN,LOW);

  Serial.print("Latitude: ");
  Serial.println(latitude, 6);  // Print latitude with 6 decimal places
  Serial.print("Longitude: ");
  Serial.println(longitude, 6);  // Print longitude with 6 decimal places
  StaticJsonDocument<500> doc;
  doc["temperature"] = random(300);
  doc["voltage"] = milliVolts;
  doc["latitude"] = latitude;
  doc["longitude"] = longitude;
  doc["Time"] = dateTime;
  

  // Serialize the JSON document to a buffer
  char buffer[500];
  serializeJson(doc, buffer);

  // Now you can use the 'buffer' variable containing the JSON message

  //    while(Serial.available()) message+=(char)Serial.read();
  mqtt.publish(topicOut, buffer);
  Serial.println("sended ! ");

  delay(1000);

  if (mqtt.connected()) {
    mqtt.loop();
  }
}

boolean mqttConnect() {
  if (!mqtt.connect("Locker", "6msn2gidAjFdLxc00d7m", "")) {
    Serial.print(".");
    return false;
  }
  Serial.println("Connected to broker.");
  mqtt.subscribe(topicIn);
  return mqtt.connected();
}


void mqttCallback(char* topic, byte* payload, unsigned int len) {
  Serial.print("Message receive:");
  Serial.write(payload, len);
  Serial.println();

  // Parse the payload as a JSON object
  DynamicJsonDocument doc(512);
  deserializeJson(doc, payload, len);

  // Check if the "key" exists in the JSON object
  if (doc.containsKey("key")) {
    // Get the value of "key"
    bool keyState = doc["key"];

    // Depending on the value of "key", control the PowerSwitch
    if (keyState) {
      Serial.println("RaspberryPi is On");
    } else {
      Serial.println("RaspberryPi is Off");
    }
  } else {
    Serial.println("Error: 'key' not found in the JSON payload");
  }
}
