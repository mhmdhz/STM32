#include <ArduinoJson.h>
#include <HardwareSerial.h>


#define SERIAL_DEBUG Serial5
#define SERIAL_RPI Serial3

HardwareSerial Serial3(PD_9, PD_8);
HardwareSerial Serial5(PD_2, PC_12);


String receivedMessage = "";
unsigned long previousDataMillis = 0;
const long dataInterval = 2000;
unsigned long previousRequestMillis = 0;
const long requestInterval = 6000; 

void sendRegularData();
void sendRequestData();
void handleUARTData();

void setup() {
  SERIAL_DEBUG.begin(115200);      
  SERIAL_RPI.begin(115200); 
  SERIAL_DEBUG.println("*************** Initialize The UART Started ***************");
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousDataMillis >= dataInterval) {
    previousDataMillis = currentMillis;
    sendRegularData();
  }

  if (currentMillis - previousRequestMillis >= requestInterval) {
    previousRequestMillis = currentMillis;
    sendRequestData();
  }


}
void sendRegularData() {
  StaticJsonDocument<100> doc;
  doc["speed"] = random(50,100);
  doc["RPM"] = random(800,3000);
  doc["timestamp"] = millis();

  String jsonString;
  serializeJson(doc, jsonString);

  SERIAL_RPI.println(jsonString);
  SERIAL_DEBUG.print("Sent Regular Data JSON: ");
  SERIAL_DEBUG.println(jsonString);
}

void sendRequestData() {
  StaticJsonDocument<50> doc;
  doc["RequestData"] =  true;
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
    String line = SERIAL_RPI.readStringUntil('\n');
    SERIAL_DEBUG.println(line);
}
