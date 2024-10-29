#include "Arduino.h"
#include <ArduinoJson.h>
#include <HardwareSerial.h>
// #include "ELMduino.h"
#include "PIDs.h"

#define TINY_GSM_MODEM_SIM808
#include <TinyGsmClient.h>
#include <PubSubClient.h>


#define SERIAL_OBD Serial1
#define SERIAL_GSM Serial2
#define SERIAL_RPI Serial3
#define SERIAL_DEBUG Serial5



#define DEBUG_BAUD 115200
#define OBD_BAUD 10400
#define RPI_BAUD 115200
#define GSM_BAUD 115200

#define LED_PIN_PE5 PE5  // Example pin; change as needed
#define LED_PIN_PC2 PC2
#define SIM808_POWERKEY PC10
#define RPI_Switch PE3
#define analogInputPin PA1
#define analogInputPinCar PA6
#define K_line_RX PA_9
#define K_line_TX PA_10
#define READ_DELAY 5
int REQUEST_DELAY;
// String protocol = "Automatic";
// String protocol = "ISO9141";
// String protocol = "ISO14230_Slow";
String protocol = "ISO14230_Fast";


HardwareSerial Serial1(K_line_TX, K_line_RX);  // Connected to OBD2 car diagnostic PORT TX, RX
HardwareSerial Serial2(PA_3, PA_2);  // Connected to SIM808 TX2,RX2
HardwareSerial Serial3(PD_9, PD_8);
HardwareSerial Serial5(PD_2, PC_12);

String dateTime = "";
String CarBattryStatus = "";
String receivedMessage = "";
String OBDerror = "";

unsigned long previousDataMillis = 0;
const long dataInterval = 2000;
unsigned long previousRequestMillis = 0;
const long requestInterval = 6000;

unsigned long previuseCheckGSMMillis = 0;
const long CheckGSMInterval = 10000;

unsigned long previuseSendDataMillis = 0;
const long SendDataInterval = 10000;

unsigned long previuseSendGPSMillis = 0;
const long SendGPSInterval = 10000;

unsigned long lastRPMtime = 0;
const unsigned long IntervalRPMtime = 120000;  // 10 minutes
float rpm = 0;
float speed = 0.123;
float latitude = 0.0;
float longitude = 0.0;
const float referenceVoltage = 12;
bool obdpower = false;
bool PowerSwitchState = false;
bool processDataFromUART = false;
float voltage = 0;
float no_face = 0;
float distraction = 0;
float sleep = 0;
float cpu_usage = 0;
float cpu_temp = 0;
bool trytoConnect = false;
bool SendDataEnable = false;

float CarDashBoardVoltage = 0;
float CarDashBoardVoltage2 = 0;
float sensorValue = 0;
const char apn[] = "mcinet";
const char user[] = "";
const char pass[] = "";
const char* broker = "62.60.211.87";
const char* topicOut = "v1/devices/me/telemetry";
const char* topicIn = "v1/devices/me/attributes";



int fuelSystemStatus = 0, engineLoadValue = 0, engineCoolantTemp = 0, shortTermFuelTrimBank1 = 0;
int longTermFuelTrimBank1 = 0, shortTermFuelTrimBank2 = 0, longTermFuelTrimBank2 = 0, fuelPressureValue = 0;
int intakeManifoldAbsPressure = 0, engineRpmValue = 0, vehicleSpeedValue = 0, timingAdvanceValue = 0;
int intakeAirTempValue = 0,  throttlePositionValue = 0, secondaryAirStatus = 0;
int oxygenSensorsPresent2Banks = 0, oxygenSensor1Voltage = 0, shortTermFuelTrim1 = 0, oxygenSensor2Voltage = 0;
int shortTermFuelTrim2 = 0, oxygenSensor3Voltage = 0, shortTermFuelTrim3 = 0, oxygenSensor4Voltage = 0;
int shortTermFuelTrim4 = 0, oxygenSensor5Voltage = 0, shortTermFuelTrim5 = 0, oxygenSensor6Voltage = 0;
int shortTermFuelTrim6 = 0, oxygenSensor7Voltage = 0, shortTermFuelTrim7 = 0, oxygenSensor8Voltage = 0;
int shortTermFuelTrim8 = 0, obdStandards = 0, oxygenSensorsPresent4Banks = 0, auxiliaryInputStatus = 0;
int runTimeSinceEngineStart = 0, distanceWithMilOn = 0;
float mafAirFlowRate = 0;

int freeze_SPEED = 0, freeze_RPM = 0, freeze_COOLANT_TEMP = 0, freeze_ENGINELOAD = 0;
String Vehicle_VIN = "", Vehicle_ID = "", Vehicle_ID_Num = "";

bool KLineStatus = false;

// ELM327 elm;
TinyGsm modem(SERIAL_GSM);
TinyGsmClient client(modem);
PubSubClient mqtt(client);

void sendRegularData();
void sendRequestData();
void handleUARTData();
void indicateError();
void indicateSuccess();
void SendGPSData();
void SendALLData();
bool OBD2_DATA();
bool retryFunction(bool (*func)(), int maxAttempts, unsigned long maxTotalTime);

void setup() {
  delay(100);
  SERIAL_DEBUG.begin(DEBUG_BAUD);
  SERIAL_RPI.begin(RPI_BAUD);

  pinMode(K_line_RX, INPUT_PULLUP);
  pinMode(K_line_TX, OUTPUT);

  SERIAL_OBD.begin(OBD_BAUD);
  SERIAL_GSM.begin(GSM_BAUD);
  pinMode(SIM808_POWERKEY, OUTPUT);
  digitalWrite(SIM808_POWERKEY, LOW);
  pinMode(LED_PIN_PE5, OUTPUT);
  digitalWrite(LED_PIN_PE5, LOW);
  pinMode(RPI_Switch, OUTPUT);
  digitalWrite(RPI_Switch, LOW);
  delay(100);
  SERIAL_DEBUG.println("Modem: " + modem.getModemInfo());
  SERIAL_DEBUG.println("Searching for telco provider.");
  SERIAL_DEBUG.println("Enabling battery charging...");
  modem.sendAT("+ECHARGE=1");
  String response = modem.stream.readStringUntil('\n');
  SERIAL_DEBUG.println(response);
  SERIAL_DEBUG.println("Checking battery status...");
  modem.sendAT("+CBC");  // Get battery status
  String Response = modem.stream.readStringUntil('\n');
  SERIAL_DEBUG.println(Response);
  mqtt.setServer(broker, 1883);
  mqtt.setCallback(mqttCallback);
  SERIAL_DEBUG.println("Connecting to GSM .");
  gsmInitialize();
  delay(100);
  modem.enableGPS();
  delay(100);
  // connectToELM327();
  SERIAL_DEBUG.println("*************** INITIALIZE THE ERO SYSTEM SETUP STARTED ***************");
}


void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousDataMillis >= dataInterval && processDataFromUART) {
    previousDataMillis = currentMillis;
    modem.getGPS(&latitude, &longitude, &speed);
    sendRegularData();

    sensorValue = analogRead(analogInputPin);
    voltage = sensorValue * (referenceVoltage / 1023.0);

    CarDashBoardVoltage2 = analogRead(analogInputPinCar);
    CarDashBoardVoltage = CarDashBoardVoltage2 * (referenceVoltage / 1023.0);
    SERIAL_DEBUG.print("CarDashBoardVoltage: ");
    SERIAL_DEBUG.println(CarDashBoardVoltage);
  }
  if (currentMillis - previousRequestMillis >= requestInterval && processDataFromUART) {
    previousRequestMillis = currentMillis;
    sendRequestData();
  }

  if (currentMillis - previuseCheckGSMMillis >= CheckGSMInterval && trytoConnect) {
    previuseCheckGSMMillis = currentMillis;
    if (checkGsmConection()) {
      SERIAL_DEBUG.println("GSM is conected ");
    } else {
      SERIAL_DEBUG.println("GSM is not  conected");
    }
  }

  if (currentMillis - previuseSendDataMillis >= SendDataInterval) {
    previuseSendDataMillis = currentMillis;
    SendDataEnable = true;
  }


  if (currentMillis - previuseSendGPSMillis >= SendGPSInterval) {
    previuseSendGPSMillis = currentMillis;
    sensorValue = analogRead(analogInputPin);
    voltage = sensorValue * (referenceVoltage / 1023.0);

    CarDashBoardVoltage2 = analogRead(analogInputPinCar);
    CarDashBoardVoltage = CarDashBoardVoltage2 * (referenceVoltage / 1023.0);
    SERIAL_DEBUG.print("CarDashBoardVoltage: ");
    SERIAL_DEBUG.println(CarDashBoardVoltage);
    dateTime = modem.getGSMDateTime(DATE_TIME);
    // SERIAL_DEBUG.print(" ---- > voltage  :  ");
    // SERIAL_DEBUG.print(voltage);
    SERIAL_DEBUG.print(" ---- > Time  :  ");
    SERIAL_DEBUG.println(dateTime);

    modem.getGPS(&latitude, &longitude, &speed);
    SERIAL_DEBUG.print("Latitude: ");
    SERIAL_DEBUG.println(latitude, 6);  // Print latitude with 6 decimal places
    SERIAL_DEBUG.print("Longitude: ");
    SERIAL_DEBUG.println(longitude, 6);  // Print longitude with 6 decimal places
    SERIAL_DEBUG.print("Speed: ");
    SERIAL_DEBUG.println(speed, 6);  // Print longitude with 6 decimal places
    SERIAL_DEBUG.print("Km/h: ");
    SERIAL_DEBUG.println((speed)*3.6, 6);  // Print longitude with 6 decimal places
    SendGPSData();
  }
  // OBD2_DATA();

  // float sensorValue = analogRead(analogInputPin);
  // voltage = sensorValue * (referenceVoltage / 1023.0);

  // CarDashBoardVoltage2 = analogRead(analogInputPinCar);
  // CarDashBoardVoltage = CarDashBoardVoltage2 * (referenceVoltage / 1023.0);

  // SERIAL_DEBUG.print("CarDashBoardVoltage: ");
  //     SERIAL_DEBUG.println(CarDashBoardVoltage);
  // delay(100);
  // if (processDataFromUART) {
  //   sendRegularData();
  //   delay(10);
  //   sendRequestData(); // to do
  // }


  if (voltage > 9) {
    obdpower = true;
    CarBattryStatus = "ENOUGH OBDII POWER";
    // OBD2_DATA();
    if (retryFunction(OBD2_DATA, 1, 1000)) {
        SERIAL_DEBUG.println("OBDII LOOP successful.");
    } else {
        SERIAL_DEBUG.println("OBDII LOOP failed after retries.");
    }

    if (CarDashBoardVoltage > 1 || PowerSwitchState) {
      digitalWrite(RPI_Switch, HIGH);
      processDataFromUART = true;
      if (SendDataEnable) { SendALLData(); }
    }

    else if (CarDashBoardVoltage < 1 && PowerSwitchState) {
      digitalWrite(RPI_Switch, HIGH);
      processDataFromUART = true;
      if (SendDataEnable) { SendALLData(); }
    }

    if (CarDashBoardVoltage < 1 && !PowerSwitchState) {  //car off and server command off wait for 2 min to turn off RPI
      digitalWrite(RPI_Switch, LOW);
      processDataFromUART = false ;
      unsigned long currentrpmTime = millis();
      if (currentrpmTime - lastRPMtime >= IntervalRPMtime) {
        digitalWrite(RPI_Switch, LOW);
        processDataFromUART = false;
        lastRPMtime = currentrpmTime;
      }
    }
  } else if (voltage < 5.5) {
    CarBattryStatus = "!! LOW OBDII POWER !!";
    if (SendDataEnable) { SendALLData(); }
  } else if (voltage < 2) {
    digitalWrite(RPI_Switch, LOW);
    obdpower = false;
    processDataFromUART = false;
    SendGPSData();

  }

  else {
    SendGPSData();
    digitalWrite(RPI_Switch, LOW);
    processDataFromUART = false;
  }


  mqtt.loop();
}

void SendALLData() {

  StaticJsonDocument<500> doc;
  doc["RPM"] = engineRpmValue;
  doc["speed"] = speed;
  doc["speed_OBDII"] = vehicleSpeedValue;  
  doc["engineCoolantTemp"] = engineCoolantTemp;
  doc["intakeAirTempValue"] = intakeAirTempValue;
  doc["throttlePositionValue"] = throttlePositionValue;
  doc["mafAirFlowRate"] = mafAirFlowRate;
  doc["engineLoadValue"] = engineLoadValue;


  doc["sleep_time"] = sleep;
  doc["noface_time"] = no_face;
  doc["distraction_time"] = distraction;
  doc["cpu_usage"] = cpu_usage;
  doc["cpu_temp"] = cpu_temp;
  doc["latitude"] = latitude;
  doc["longitude"] = longitude;
  doc["systemVoltage"] = voltage;
  doc["OBD2_Power_connection"] = obdpower;
  doc["Time"] = dateTime;
  doc["CarBattryStatus"] = CarBattryStatus;
  doc["OBD_ERROR"] = OBDerror;

  char buffer[500];
  serializeJson(doc, buffer);


  if (mqtt.publish(topicOut, buffer) == true) {
    trytoConnect = false;
    SendDataEnable = false;
    SERIAL_DEBUG.println("Sent ALL DATA To Server");
    SERIAL_DEBUG.println("MQTT state: " + String(mqtt.state()));

  } else {
    SERIAL_DEBUG.println("Failed to send data. MQTT state: " + String(mqtt.state()));

    // Print detailed connection state
    SERIAL_DEBUG.print("MQTT state: ");
    SERIAL_DEBUG.println(mqtt.state());
    trytoConnect = true;

    // Additional retry logic can be added here if needed
  }
  // delay(300);
}
void SendGPSData() {

  StaticJsonDocument<200> doc;
  doc["latitude"] = latitude;
  doc["longitude"] = longitude;
  doc["CarBattryStatus"] = CarBattryStatus;
  doc["Time"] = dateTime;
  doc["OBD_ERROR"] = OBDerror;



  char buffer[200];

  serializeJson(doc, buffer);
  if (mqtt.publish(topicOut, buffer) == true) {
    trytoConnect = false;
    SERIAL_DEBUG.println("Sended GPS DATA To Server ");

  } else {
    SERIAL_DEBUG.println("Failed GPS DATA to send data.");
    trytoConnect = true;
  }
}
bool OBD2_DATA() {
  // SERIAL_DEBUG.println(elm.connected);
  if (KLineStatus == false) {
    Serial.println("Initialising...");
    bool init_success = init_OBD2();

    if (init_success) {
      KLineStatus = true;
      digitalWrite(LED_PIN_PE5, HIGH);
      Serial.println("Init Success !!");
    }
  } else {
    read_K();
    return true ; 
  }
}

bool checkGsmConection() {

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

  return true;
}
void sendRegularData() {
  StaticJsonDocument<100> doc;
  doc["speed"] = speed;
  doc["speed_OBDII"] = vehicleSpeedValue;

  doc["RPM"] = engineRpmValue;


  doc["timestamp"] = millis();


  String jsonString;
  serializeJson(doc, jsonString);

  SERIAL_RPI.println(jsonString);
  SERIAL_DEBUG.print("Sent Regular Data JSON: ");
  SERIAL_DEBUG.println(jsonString);
}

void sendRequestData() {
  StaticJsonDocument<50> doc;
  doc["RequestData"] = true;
  String jsonString;
  serializeJson(doc, jsonString);
  SERIAL_RPI.println(jsonString);
  // delay(3000);
  handleUARTData();
  // SERIAL_DEBUG.print("Sent RequestData JSON: ");
  // SERIAL_DEBUG.println(jsonString);
}

void handleUARTData() {
  String incomingMessage = "";
  // delay(100);
  String line = SERIAL_RPI.readStringUntil('\n');
  // delay(10);
  SERIAL_DEBUG.println(line);

  // {"no_face": 377.54626179553327, "distraction": 7.5558234273613305, "sleep": 0, "cpu_usage": 16.8, "cpu_temp": 51121}

  StaticJsonDocument<500> doc;
  DeserializationError error = deserializeJson(doc, line);

  if (error) {
    SERIAL_DEBUG.print(F("deserializeJson() failed: "));
    SERIAL_DEBUG.println(error.f_str());
    return;
  }
  no_face = doc["no_face"];
  distraction = doc["distraction"];
  sleep = doc["sleep"];
  cpu_usage = doc["cpu_usage"];
  cpu_temp = doc["cpu_temp"];
}

void indicateError() {
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_PIN_PE5, HIGH);
    delay(50);
    digitalWrite(LED_PIN_PE5, LOW);
    delay(50);
    OBDerror = "FAILED";
    // SendGPSData();
  }
}

void indicateSuccess() {
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_PIN_PE5, HIGH);
    delay(200);
    digitalWrite(LED_PIN_PE5, LOW);
    delay(200);
    // OBDerror = "SUCCESS";
  }
}


// void connectToELM327() {
//   if (elm.begin(SERIAL_OBD)) {
//     SERIAL_DEBUG.println("Connected to ELM327 successfully.");
//     indicateSuccess();
//   } else {
//     SERIAL_DEBUG.println("Failed to connect to ELM327.");
//     indicateError();
//   }
// }

bool mqttConnect() {
  if (!mqtt.connect("ERO_drivermonitoring_system", "ohf1t09gdut5l0erx9oa", "")) {
    SERIAL_DEBUG.print("Failed MQTT connection, rc=");
    SERIAL_DEBUG.print(mqtt.state());
    SERIAL_DEBUG.println(" try again in 10 miliseconds");
    delay(10);
    return false;
  }
  SERIAL_DEBUG.println("Connected to MQTT Broker.");
  mqtt.subscribe(topicIn);
  return mqtt.connected();
}

void mqttCallback(char* topic, byte* payload, unsigned int len) {
  SERIAL_DEBUG.print("Message received on topic ");
  SERIAL_DEBUG.print(topic);
  SERIAL_DEBUG.print(": ");
  SERIAL_DEBUG.write(payload, len);
  SERIAL_DEBUG.println();

  // Parse the payload as a JSON object
  DynamicJsonDocument doc(512);
  DeserializationError error = deserializeJson(doc, payload, len);
  if (error) {
    SERIAL_DEBUG.print("Failed to parse MQTT message: ");
    SERIAL_DEBUG.println(error.c_str());
    return;
  }

  // Handle "key" in JSON
  if (doc.containsKey("key")) {
    bool keyState = doc["key"];

    if (keyState) {
      digitalWrite(LED_PIN_PC2, LOW);
      SERIAL_DEBUG.println("RaspberryPi is On");
      PowerSwitchState = true;
    } else {
      digitalWrite(LED_PIN_PC2, HIGH);
      SERIAL_DEBUG.println("RaspberryPi is Off");
      PowerSwitchState = false;
    }
  } else {
    SERIAL_DEBUG.println("Error: 'key' not found in the JSON payload");
  }

  // Handle "rpm" in JSON
  if (doc.containsKey("rpm")) {
    rpm = doc["rpm"];
    SERIAL_DEBUG.println(rpm);
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
    SERIAL_DEBUG.print("Attempt ");
    SERIAL_DEBUG.print(attempts);
    SERIAL_DEBUG.println(" failed. Retrying...");

    // Optional: Wait before retrying
    delay(500);  // 1 second delay between attempts
  }
  return false;  // All attempts failed or timeout exceeded
}


bool tryNetworkConnection() {
  if (modem.waitForNetwork()) {
    SERIAL_DEBUG.println("Network connected .");

    return true;  // Success
  }
  SERIAL_DEBUG.println("Network connection failed.");
  return false;  // Failure
}

bool tryGPRSConnection() {
  if (modem.gprsConnect(apn, user, pass)) {
    SERIAL_DEBUG.println("GPRS connected .");

    return true;  // Success
  }
  SERIAL_DEBUG.println("GPRS connection failed.");
  return false;  // Failure
}

bool tryMQTTConnection() {
  return mqttConnect();
}

bool gsmInitialize() {
  // Try to connect to the network
  if (!retryFunction(tryNetworkConnection, 3, 10000)) {
    // Return false if network connection fails
    return false;
  } else {
    // Signal Quality and GPRS Connection
    SERIAL_DEBUG.println("Signal Quality: " + String(modem.getSignalQuality()));
    SERIAL_DEBUG.println("Connected to network.");
  }
  // Try to connect to GPRS
  if (!retryFunction(tryGPRSConnection, 3, 10000)) {
    // Return false if GPRS connection fails
    return false;
  } else {
    SERIAL_DEBUG.println("Connected to GPRS: " + String(apn));
    delay(10);
  }


  // Try to connect to the MQTT Broker
  if (!retryFunction(tryMQTTConnection, 3, 10000)) {
    // Return false if MQTT connection fails
    return false;
  } else {
    SERIAL_DEBUG.println("Connected to MQTT Broker: " + String(broker));
    delay(10);
  }
  // If all connections succeed, return true
  return true;
}
