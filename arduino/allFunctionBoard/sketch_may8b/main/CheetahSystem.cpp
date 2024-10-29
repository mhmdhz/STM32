#include "CheetahSystem.h"

// Initialize the static instance pointer to nullptr
CheetahSystem* CheetahSystem::instance = nullptr;

// Constructor
CheetahSystem::CheetahSystem()
  : OBD2_PORT(PA_10, PA_9),
    SerialAT(PA_3, PA_2),
    Raspberrypi(PD_9, PD_8),
    SERIAL_PORT(PD_2, PC_12),
    modem(SerialAT),
    gsmClient(modem),
    mqttClient(gsmClient),
    mqttHandler(mqttClient),
    obdHandler(OBD2_PORT, SERIAL_PORT),
    powerManager(PowerSwitchPin, LedPin, Led2Pin, buzzerPin, SIM808_POWERKEYPin),
    lastPublishGpsTime(0),
    lastPublishrpmTime(0),
    processDataFromUART(true),
    PowerSwitchState(false),
    obdpower(true),
    latitude(0.0),
    longitude(0.0),
    rpm(0),
    speed(0),
    voltage(0),
    dateTime(""),
    batteryStatus("")
{
  // Assign the instance pointer to this object
  instance = this;
}

// Initialize all modules and settings
void CheetahSystem::initialize() {
  // Initialize Serial Ports
  SERIAL_PORT.begin(115200);
  SerialAT.begin(115200);
  OBD2_PORT.begin(38400);
  Raspberrypi.begin(115200);
  
  SERIAL_PORT.println("started !");
  
  // Initialize Power Manager
  powerManager.begin();
  
  // Initialize Modem
  modem.restart();
  delay(100);
  SERIAL_PORT.println("Modem: " + modem.getModemInfo());
  
  SERIAL_PORT.println("Searching for telco provider.");
  if (!modem.waitForNetwork()) {
    SERIAL_PORT.println("Failed to connect to network.");
    while (true);
  }
  
  SERIAL_PORT.println("Connected to telco.");
  SERIAL_PORT.println("Signal Quality: " + String(modem.getSignalQuality()));
  
  SERIAL_PORT.println("Connecting to GPRS network.");
  if (!modem.gprsConnect(apn, user, pass)) {
    SERIAL_PORT.println("Failed to connect to GPRS.");
    while (true);
  }
  
  SERIAL_PORT.println("Connected to GPRS: " + String(apn));
  
  // Initialize MQTT
  mqttClient.setServer(broker, 1883);
  mqttClient.setCallback(staticMqttCallback); // Use static callback

  SERIAL_PORT.println("Connecting to MQTT Broker: " + String(broker));
  while (!mqttConnect()) {
    delay(1000);
  }
  SERIAL_PORT.println("Connected to MQTT Broker.");
  
  // Initialize ELM327
  if (!obdHandler.begin()) {
    SERIAL_PORT.println("Failed to connect to ELM327");
    digitalWrite(LedPin, HIGH);
    while (1);
  }
  
  SERIAL_PORT.println("Connected to ELM327");
  
  // Blink LED to indicate startup
  for (int i = 0; i < 20; i++) {
    digitalWrite(LedPin, HIGH);
    delay(100);
    digitalWrite(LedPin, LOW);
    delay(100);
  }
}

// Main loop
void CheetahSystem::run() {
  // Handle MQTT connection
  if (!mqttClient.connected()) {
    while (!mqttConnect()) {
      delay(1000);
    }
  }
  mqttClient.loop();

  // Read OBD Data
  rpm = obdHandler.getRPM();
  speed = obdHandler.getSpeed();

  // Read Voltage
  voltage = powerManager.readVoltage(analogInputPin, referenceVoltage);

  // Get Date and Time from GSM
  dateTime = modem.getGSMDateTime(DATE_TIME);

  SERIAL_PORT.print(millis());
  SERIAL_PORT.print(" ---- > voltage  :  ");
  SERIAL_PORT.print(voltage);
  SERIAL_PORT.print(" ---- > Time  :  ");
  SERIAL_PORT.println(dateTime);

  // Get GPS Data
  modem.getGPS(&latitude, &longitude);
  SERIAL_PORT.print("Latitude: ");
  SERIAL_PORT.println(latitude, 6);
  SERIAL_PORT.print("Longitude: ");
  SERIAL_PORT.println(longitude, 6);

  delay(500);

  // Power Management Logic
  if (voltage > 11.00) {
    obdpower = true;
    batteryStatus = "ENOUGH POWER";
    if (rpm > 100 || PowerSwitchState) {
      processDataFromUART = true;
      powerManager.turnOnPower();
      delay(500);
    }
    else if (rpm == 0.0 && PowerSwitchState) {
      powerManager.turnOnPower();
      delay(500);
    }

    if (rpm == 0.0 && !PowerSwitchState) {
      unsigned long currentrpmTime = millis();
      if (currentrpmTime - lastPublishrpmTime >= publishIntervalrpm) {
        powerManager.turnOffPower();
        lastPublishrpmTime = currentrpmTime;
      }
    }
  } 
  else if (voltage < 10.5) {
    batteryStatus = "!! LOW POWER !!";
  } 
  else {
    obdpower = false;
  }

  // Handle UART Data from Raspberry Pi
  if (processDataFromUART) {
    if (Raspberrypi.available()) {
      while (Raspberrypi.available() > 0) {
        char incomingByte = Raspberrypi.read();
        if (incomingByte != '\n') {
          receivedMessage += incomingByte;
        } else {
          mqttHandler.parseAndPublishData(receivedMessage.c_str(), 
            rpm, speed, latitude, longitude, obdpower, dateTime, batteryStatus);
          receivedMessage = "";
        }
      }
    } else {
      mqttHandler.publishData(rpm, speed, latitude, longitude, voltage, obdpower, dateTime, batteryStatus);
    }
  } else {
    unsigned long currentGpsTime = millis();
    if (currentGpsTime - lastPublishGpsTime >= publishIntervalGps) {
      mqttHandler.publishLatLong(latitude, longitude);
      lastPublishGpsTime = currentGpsTime;
    }
  }
}

// Member MQTT Callback
void CheetahSystem::mqttCallback(char* topic, byte* payload, unsigned int len) {
  mqttHandler.handleMessage(topic, payload, len, PowerSwitchState, Led2Pin, SERIAL_PORT);
}

// Static MQTT Callback
void CheetahSystem::staticMqttCallback(char* topic, byte* payload, unsigned int len) {
  if (instance != nullptr) {
    instance->mqttCallback(topic, payload, len);
  }
}

// MQTT Connect
boolean CheetahSystem::mqttConnect() {
  return mqttHandler.connect("test-system", "gueqfjegm2ecpb1tt22s", "");
}
