#define TINY_GSM_MODEM_SIM808
#include "Arduino.h"
#include "ELMduino.h"
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// Existing Definitions
ELM327 elm;
// #define OBD_Serial Serial1
#define OBD2_PORT Serial1
#define SerialAT Serial2
#define Raspberrypi Serial3
#define SERIAL_PORT Serial5

HardwareSerial Serial1(PA_10, PA_9);  // Connected to OBD2 car diag PORT
HardwareSerial Serial2(PA_3, PA_2);   // Connected to SIM808 TX2,RX2
HardwareSerial Serial3(PD_9, PD_8);   // Connected to Raspberry Pi (TX3, RX3)
HardwareSerial Serial5(PD_2, PC_12);  // Connected to TTL to USB for serial debugging TX3,RX3

#define analogInputPin PA1
#define PowerSwitch PE3
#define buzzer PE9
#define Led PE5
#define Led2 PC2
#define SIM808_POWERKEY PC10
#define FAN_PWM PA6
#define IR_PWM PB14

bool processDataFromUART = true;
bool PowerSwitchState = false;
bool obdpower = true;
const float referenceVoltage = 12;
float latitude = 0.0;
float longitude = 0.0;
float rpm = 1000;
// float rpm = 0;
float speed = 0;
int valueCount = 0;
float voltage = 0;
String receivedMessage = "";
String dateTime = "";
String batteryStatus = "";

unsigned long lastPublishGpsTime = 0;
const unsigned long publishIntervalGps = 6000;  // 10 minutes
unsigned long lastPublishrpmTime = 0;
const unsigned long publishIntervalrpm = 20000;  // 20 seconds

// Network details
const char apn[] = "mcinet";
const char user[] = "";
const char pass[] = "";

// MQTT details
const char* broker = "thingsboard.cloud";
const char* topicOut = "v1/devices/me/telemetry";
const char* topicIn = "v1/devices/me/attributes";

TinyGsm modem(SerialAT);
TinyGsmClient client(modem);
PubSubClient mqtt(client);

// === Buzzer State Machine Definitions ===
enum BuzzerState {
  IDLE,
  BEEP_ON,
  BEEP_OFF
};

BuzzerState buzzerState = IDLE;

// Buzzer control variables
unsigned long buzzerPreviousMillis = 0;
const unsigned long buzzerOnDuration = 100;   // Buzzer ON for 100 ms (fast beep)
const unsigned long buzzerOffDuration = 100;  // Buzzer OFF for 100 ms
int buzzerBeepCount = 0;
const int totalBeeps = 3;

// === Function Prototypes ===
void parseAndPublishData(const char* message);
void publishData();
void publishLatLong();
void mqttCallback(char* topic, byte* payload, unsigned int len);
boolean mqttConnect();
void handleBuzzer();

// === Setup Function ===
void setup() {
  delay(100);
  SERIAL_PORT.begin(115200);  // Set baud rate for OBD communication
  delay(100);
  SERIAL_PORT.println();
  SERIAL_PORT.println("*************** Initialize The System Started ***************");
  SerialAT.begin(115200);
  OBD2_PORT.begin(38400);
  Raspberrypi.begin(9600);

  // Initialize Pins
  pinMode(Led, OUTPUT);
  pinMode(Led2, OUTPUT);
  pinMode(PowerSwitch, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(SIM808_POWERKEY, OUTPUT);

  // Initialize Pin States
  digitalWrite(SIM808_POWERKEY, LOW);
  delay(1000);
  digitalWrite(SIM808_POWERKEY, HIGH);
  digitalWrite(Led, LOW);
  digitalWrite(Led2, HIGH);
  digitalWrite(PowerSwitch, LOW);
  digitalWrite(buzzer, LOW);

  // Initialize Modem
  modem.restart();
  delay(100);

  // Modem Information
  SERIAL_PORT.println("Modem: " + modem.getModemInfo());
  SERIAL_PORT.println("Searching for telco provider.");

  if (!modem.waitForNetwork()) {
    SERIAL_PORT.println("Network connection failed.");
    while (true)
      ;
  }

  SERIAL_PORT.println("Connected to telco.");

  // Battery Charging
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

  if (!modem.gprsConnect(apn, user, pass)) {
    SERIAL_PORT.println("GPRS connection failed.");
    while (true)
      ;
  }

  SERIAL_PORT.println("Connected to GPRS: " + String(apn));
  digitalWrite(Led, LOW);

  // MQTT Setup
  mqtt.setServer(broker, 1883);
  mqtt.setCallback(mqttCallback);

  SERIAL_PORT.println("Connecting to MQTT Broker: " + String(broker));

  // while (mqttConnect() == false) continue;

  SERIAL_PORT.println("Connected to MQTT Broker.");
  delay(1000);

  SERIAL_PORT.println("GPS Truning ON ...");
  modem.enableGPS();
  delay(500);
  SERIAL_PORT.println("GPS Activated .");



  if (elm.begin(OBD2_PORT)) {
    SERIAL_PORT.println("Connected to OBDII successfully.");
    digitalWrite(Led, HIGH);
    delay(100);
    digitalWrite(Led, LOW);
    delay(100);
    // SERIAL_PORT.println("Connected to OBDII ");

  } else {
    SERIAL_PORT.println("Connection failed to OBDII Please Check the connection");
  }

  SERIAL_PORT.println("Connected to CAR  ");
  SERIAL_PORT.println("Wait 2 Secound to Start ERO Driver Monitoring System ");


  // Blink LED to indicate startup
  for (int i = 0; i < 20; i++) {
    digitalWrite(Led, HIGH);
    delay(100);
    digitalWrite(Led, LOW);
    delay(100);
  }

  SERIAL_PORT.println("*************** DONE  ***************");
}

// === Main Loop ===
void loop() {
  // Ensure MQTT is connected
  if (!mqtt.connected()) {
    while (!mqttConnect()) {
      delay(100);
    }
  }
  mqtt.loop();

  // Update RPM and Speed
  // rpm = elm.rpm();
  // speed = elm.kph();

  // Read Voltage
  float sensorValue = analogRead(analogInputPin);
  voltage = sensorValue * (referenceVoltage / 1023.0);

  // Get Date and Time
  dateTime = modem.getGSMDateTime(DATE_TIME);

  // Debugging Information
  SERIAL_PORT.print(millis());
  SERIAL_PORT.print(" ---- > voltage  :  ");
  SERIAL_PORT.print(voltage);
  SERIAL_PORT.print(" ---- > Time  :  ");
  SERIAL_PORT.println(dateTime);

  // Get GPS Data
  modem.getGPS(&latitude, &longitude);
  SERIAL_PORT.print("Latitude: ");
  SERIAL_PORT.println(latitude, 6);  // Print latitude with 6 decimal places
  SERIAL_PORT.print("Longitude: ");
  SERIAL_PORT.println(longitude, 6);  // Print longitude with 6 decimal places

  // Handle Buzzer State Machine
  handleBuzzer();

  // Handle Power Switching based on Voltage and RPM
  if (voltage > 9) {
    obdpower = true;
    batteryStatus = "ENOUGH POWER";

    if (rpm > 100 || PowerSwitchState) {
      processDataFromUART = true;
      digitalWrite(PowerSwitch, HIGH);
      delay(1);
    } else if (rpm == 0.0 && PowerSwitchState) {
      digitalWrite(PowerSwitch, HIGH);
      delay(1);
    }

    if (rpm == 0.0 && !PowerSwitchState) {
      unsigned long currentrpmTime = millis();
      if (currentrpmTime - lastPublishrpmTime >= publishIntervalrpm) {
        digitalWrite(PowerSwitch, LOW);
        lastPublishrpmTime = currentrpmTime;
      }
    }
  } else if (voltage < 8.5) {
    batteryStatus = "!! LOW POWER !!";
  } else {
    obdpower = false;
  }

  // Handle UART Data from Raspberry Pi
  if (processDataFromUART) {
    if (Raspberrypi.available()) {
      while (Raspberrypi.available() > 0) {
        char incomingByte = Raspberrypi.read();

        if (incomingByte != '\n') {
          // Append the character to the received message
          receivedMessage += incomingByte;
        } else {
          // End of message detected, process it
          SERIAL_PORT.println(receivedMessage);
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
      lastPublishGpsTime = currentGpsTime;
    }
  }
  delay(100);
}

// === Function to Handle Buzzer State Machine ===
void handleBuzzer() {
  unsigned long currentMillis = millis();

  switch (buzzerState) {
    case IDLE:
      // Buzzer is inactive
      break;

    case BEEP_ON:
      digitalWrite(buzzer, HIGH);  // Turn buzzer ON
      buzzerPreviousMillis = currentMillis;
      buzzerState = BEEP_OFF;
      SERIAL_PORT.println("BEEB ARLARM BOOGH");
      break;

    case BEEP_OFF:
      if (currentMillis - buzzerPreviousMillis >= buzzerOnDuration) {
        digitalWrite(buzzer, LOW);  // Turn buzzer OFF
        buzzerBeepCount++;

        if (buzzerBeepCount < totalBeeps) {
          buzzerPreviousMillis = currentMillis;
          buzzerState = BEEP_ON;  // Prepare for next beep
        } else {
          buzzerState = IDLE;  // All beeps completed
          buzzerBeepCount = 0;
        }
      }
      break;
  }
}

// === Function to Process Incoming JSON and Handle Alarm ===
void parseAndPublishData(const char* message) {
  StaticJsonDocument<500> doc;
  DeserializationError error = deserializeJson(doc, message);

  if (error) {
    SERIAL_PORT.print("Failed to parse JSON: ");
    SERIAL_PORT.println(error.c_str());
    return;
  }

  // Extract and publish various fields as per original functionality
  int sleepTime = doc["sleep"];
  int noFaceTime = doc["no_face"];
  int distractionTime = doc["distraction"];
  int cpuUsage = doc["cpu_usage"];
  int cpuTemp = doc["cpu_temp"];
  unsigned long ts = doc["ts"] | 0;  // Use a default value if "ts" is missing

  StaticJsonDocument<500> publishDoc;
  publishDoc["RPM"] = rpm;
  publishDoc["speed"] = speed;
  publishDoc["sleep_time"] = sleepTime;
  publishDoc["noface_time"] = noFaceTime;
  publishDoc["distraction_time"] = distractionTime;
  publishDoc["cpu_usage"] = cpuUsage;
  publishDoc["cpu_temp"] = cpuTemp;
  publishDoc["latitude"] = latitude;
  publishDoc["longitude"] = longitude;
  publishDoc["OBD2_Power_connection"] = obdpower;
  publishDoc["Time"] = dateTime;
  publishDoc["BatteryStatus"] = batteryStatus;




  // === New: Handle Alarm from JSON ===
  if (doc.containsKey("alarm")) {
    bool alarmState = doc["alarm"];

    if (alarmState) {
      SERIAL_PORT.println("Alarm triggered: FAST BEEPING");
      // Initiate buzzer beeping if not already active
      indicateError();

      if (buzzerState == IDLE) {
        buzzerState = BEEP_ON;
        buzzerBeepCount = 0;
      }
    } else {
      SERIAL_PORT.println("Alarm cleared.");
      // Optionally, you can stop the buzzer immediately if needed
      // But since we want to beep three times, we let it finish
    }
  }
  char buffer[500];
  serializeJson(publishDoc, buffer);
  mqtt.publish(topicOut, buffer);
  if (mqtt.connected()) {
    mqtt.loop();
  }
}

// === Existing Function: publishData ===
void publishData() {
  StaticJsonDocument<500> doc;
  doc["RPM"] = rpm;
  doc["speed"] = speed;
  doc["sleep_time"] = doc["sleep_time"];  // Assuming you want to include sleep_time
  doc["noface_time"] = doc["noface_time"];
  doc["distraction_time"] = doc["distraction_time"];
  doc["cpu_usage"] = doc["cpu_usage"];
  doc["cpu_temp"] = doc["cpu_temp"];
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

// === Existing Function: publishLatLong ===
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

// === Existing Function: mqttCallback ===
void mqttCallback(char* topic, byte* payload, unsigned int len) {
  SERIAL_PORT.print("Message received on topic ");
  SERIAL_PORT.print(topic);
  SERIAL_PORT.print(": ");
  SERIAL_PORT.write(payload, len);
  SERIAL_PORT.println();

  // Parse the payload as a JSON object
  DynamicJsonDocument doc(512);
  deserializeJson(doc, payload, len);

  // Handle "key" in JSON
  if (doc.containsKey("key")) {
    bool keyState = doc["key"];

    if (keyState) {
      PowerSwitchState = true;
      digitalWrite(Led2, LOW);
      SERIAL_PORT.println("RaspberryPi is On");
    } else {
      PowerSwitchState = false;
      digitalWrite(Led2, HIGH);
      SERIAL_PORT.println("RaspberryPi is Off");
    }
  } else {
    SERIAL_PORT.println("Error: 'key' not found in the JSON payload");
  }

  // Handle "rpm" in JSON
  if (doc.containsKey("rpm")) {
    rpm = int(doc["rpm"]);
    SERIAL_PORT.println(rpm);
  }
}

// === Existing Function: mqttConnect ===
boolean mqttConnect() {
  if (!mqtt.connect("test-system1", "1oxlc4oogqobuv0vvskv", "")) {
    SERIAL_PORT.print(".");
    return false;
  }
  SERIAL_PORT.println("Connected to MQTT Broker.");
  mqtt.subscribe(topicIn);
  return mqtt.connected();
}
void indicateError() {
  
    digitalWrite(Led, HIGH);
    delay(100);
    digitalWrite(Led, LOW);
    delay(100);
    // Optionally, you can add more error indicators here
  
}