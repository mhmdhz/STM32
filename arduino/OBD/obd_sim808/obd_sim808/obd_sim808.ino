
#define TINY_GSM_MODEM_SIM808

#include "Arduino.h"
#include "ELMduino.h"
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
ELM327 elm;

#define SerialAT Serial3
// #define OBD_Serial Serial1
#define SERIAL_PORT Serial1
#define Raspberrypi Serial2

HardwareSerial Serial3(PA_10, PA_9);
HardwareSerial Serial1(PD_9, PD_8);
HardwareSerial Serial2(PA_3, PA_2);


#define analogInputPin PA1
#define PowerSwitch PE3
#define buzzer PE9
#define Led PE5
#define Led2 PC2
#define SIM808_POWERKEY PC10


bool processDataFromUART = true;
bool PowerSwitchState = false;

bool obdpower = true;
const float referenceVoltage = 12;
float latitude = 0.0;
float longitude = 0.0;
float rpm = 0;
float speed = 0;
int valueCount = 0;
float voltage = 0;
String receivedMessage = "";
String dateTime = "";
String batteryStatus = "";

unsigned long lastPublishGpsTime = 0;
const unsigned long publishIntervalGps = 600000;

unsigned long lastPublishrpmTime = 0;
const unsigned long publishIntervalrpm = 20000;
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

void parseAndPublishData(const char* message) {
  StaticJsonDocument<500> doc;

  DeserializationError error = deserializeJson(doc, message);

  if (error) {
    Serial.print("Failed to parse JSON: ");
    Serial.println(error.c_str());
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
  mqtt.publish(topicOut, buffer);
  if (mqtt.connected()) {
    mqtt.loop();
  }
}

void publishData() {
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

  mqtt.publish(topicOut, buffer);
  if (mqtt.connected()) {
    mqtt.loop();
  }
}

void publishLatLong() {
  StaticJsonDocument<500> doc;

  doc["latitude"] = latitude;
  doc["longitude"] = longitude;


  char buffer[500];
  serializeJson(doc, buffer);
  mqtt.publish(topicOut, buffer);
  if (mqtt.connected()) {
    mqtt.loop();
  }
}

// void readGPSData() {
//   // Check if GPS fix is available
//   if (!client.getGPS(&latitude, &longitude)) {
//     Serial.println("Failed to get GPS fix!");
//     return;
//   }


// }
void mqttCallback(char* topic, byte* payload, unsigned int len) {
  // SERIAL_PORT.print("Message receive:");
  // SERIAL_PORT.write(payload, len);
  // SERIAL_PORT.println();

  // Parse the payload as a JSON object
  DynamicJsonDocument doc(512);
  deserializeJson(doc, payload, len);

  // Check if the "key" exists in the JSON object
  if (doc.containsKey("key")) {
    // Get the value of "key"
    bool keyState = doc["key"];

    // Depending on the value of "key", control the PowerSwitch
    if (keyState) {
      // digitalWrite(PowerSwitch, HIGH);
      PowerSwitchState = true;
      digitalWrite(Led2, LOW);
      // SERIAL_PORT.println("RaspberryPi is On");
    } else {
      // digitalWrite(PowerSwitch, LOW);
      PowerSwitchState = false;
      digitalWrite(Led2, HIGH);
      // SERIAL_PORT.println("RaspberryPi is Off");
    }
  } else {
    // SERIAL_PORT.println("Error: 'key' not found in the JSON payload");
  }

  if (doc.containsKey("rpm")) {
    // Get the value of "RPM"
    rpm = int(doc["rpm"]);
    // SERIAL_PORT.println(rpm);
  }
}

void setup() {

  SERIAL_PORT.begin(9600);  // Set baud rate for OBD communication
  SerialAT.begin(9600);


  // SERIAL_PORT.println("started !");
  pinMode(Led, OUTPUT);
  pinMode(Led2, OUTPUT);
  pinMode(PowerSwitch, OUTPUT);
  pinMode(buzzer, OUTPUT);

  pinMode(SIM808_POWERKEY, OUTPUT);
  // put your setup code here, to run once:
  digitalWrite(SIM808_POWERKEY, 0);
  delay(1000);
  digitalWrite(SIM808_POWERKEY, 1);

  digitalWrite(Led, 0);


  // digitalWrite(Led, 1);
  digitalWrite(Led2, 1);
  digitalWrite(PowerSwitch, LOW);
  digitalWrite(buzzer, LOW);
   SERIAL_PORT.println();
   SERIAL_PORT.println("***************Cheetah System started***************");
  modem.restart();
  delay(100);
   SERIAL_PORT.println("Modem: " + modem.getModemInfo());
   SERIAL_PORT.println("Searching for telco provider.");

  if (!modem.waitForNetwork()) {
    // SERIAL_PORT.println("fail");
    while (true)
      ;
  }
   SERIAL_PORT.println("Connected to telco.");
   SERIAL_PORT.println("Signal Quality: " + String(modem.getSignalQuality()));

   SERIAL_PORT.println("Connecting to GPRS network.");
  if (!modem.gprsConnect(apn, user, pass)) {
    Serial.println("fail");
    while (true)
      ;
  }
  //   while (!client.getGPS(&latitude, &longitude)) {
  //   delay(1000);
  // }
  // SERIAL_PORT.println("Connected to GPRS: " + String(apn));
  digitalWrite(Led, 0);
  mqtt.setServer(broker, 1883);
  mqtt.setCallback(mqttCallback);

  // mqtt.setCallback(mqttCallback);
  // SERIAL_PORT.println("Connecting to MQTT Broker: " + String(broker));
  while (mqttConnect() == false) continue;
  // SERIAL_PORT.println("conected");

  delay(1000);
  modem.enableGPS();


  if (elm.begin(SERIAL_PORT)) {
    // Serial.println("Failed to connect to ELM327");
    digitalWrite(Led, 1);
    delay(100);
    digitalWrite(Led, 0);
    delay(100);
    while (1)
      ;



    // SERIAL_PORT.println("Connected to ELM327");
  }
  for (int i = 0; i < 20; i++) {
    digitalWrite(Led, 1);
    delay(100);
    digitalWrite(Led, 0);
    delay(100);
  }
}




void loop() {

  rpm = elm.rpm();
  speed = elm.kph();
  int sensorValue = analogRead(analogInputPin);
  voltage = sensorValue * (referenceVoltage / 1023.0);
  dateTime = modem.getGSMDateTime(DATE_TIME);

  // SERIAL_PORT.print(millis());
  // SERIAL_PORT.print(" ---- > voltage  :  ");
  // SERIAL_PORT.print(voltage);
  // SERIAL_PORT.print(" ---- > Time  :  ");
  // SERIAL_PORT.println(dateTime);

  modem.getGPS(&latitude, &longitude);
  //  SERIAL_PORT.println(modem.getGPSraw()) ;   // GPS data received successfully
  // SERIAL_PORT.print("Latitude: ");
  // SERIAL_PORT.println(latitude, 6);  // Print latitude with 6 decimal places
  // SERIAL_PORT.print("Longitude: ");
  // SERIAL_PORT.println(longitude, 6);  // Print longitude with 6 decimal places

  delay(500);



  if (voltage > 11.00) {
    obdpower = true;
    batteryStatus = " ENOUGH POWER ";
    if (rpm > 100 || PowerSwitchState) {
      // SERIAL_PORT.println("rpm > 100 || PowerSwitchState");
      processDataFromUART = true;
      digitalWrite(PowerSwitch, HIGH);
      delay(500);
    }

    else if (rpm == 0.0 && PowerSwitchState) {
      // SERIAL_PORT.println("rpm == 0.0 && PowerSwitchState");
      digitalWrite(PowerSwitch, HIGH);
      delay(500);
    }

    // SERIAL_PORT.println(PowerSwitchState);

    if (rpm == 0.0 && !PowerSwitchState) {
      // SERIAL_PORT.println("rpm == 0.0 && !PowerSwitchState");

      unsigned long currentrpmTime = millis();
      if (currentrpmTime - lastPublishrpmTime >= publishIntervalrpm) {
        digitalWrite(PowerSwitch, LOW);
        lastPublishrpmTime = currentrpmTime;
      }
    }
  } else if (voltage < 10.5) {
    batteryStatus = "!! LOW POWER !!";
  } else {
    obdpower = false;
  }

  if (processDataFromUART) {
    if (Raspberrypi.available()) {
      while (Raspberrypi.available() > 0) {

        char incomingByte = Raspberrypi.read();

        if (incomingByte != '\n') {
          // Append the character to the received message
          receivedMessage += incomingByte;
        } else {

          parseAndPublishData(receivedMessage.c_str());

          receivedMessage = "";
        }
      }
    } else {
      publishData();
    }
  } else {
    // If processDataFromUART is false, publish latitude and longitude every 10 minutes
    unsigned long currentGpsTime = millis();
    if (currentGpsTime - lastPublishGpsTime >= publishIntervalGps) {
      publishLatLong();
      // SERIAL_PORT.println(" Sending GPS data ! ");
      lastPublishGpsTime = currentGpsTime;
    }
  }
}
boolean mqttConnect() {
  if (!mqtt.connect("test-system", "gueqfjegm2ecpb1tt22s", "")) {
    // SERIAL_PORT.print(".");
    return false;
  }
  // SERIAL_PORT.println("Connected to broker.");
  mqtt.subscribe(topicIn);
  return mqtt.connected();
}
