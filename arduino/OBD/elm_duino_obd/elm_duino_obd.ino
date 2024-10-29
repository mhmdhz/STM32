

#include "Arduino.h"
#include "ELMduino.h"

#define SERIAL_PORT Serial3
#define OBD_SERIAL Serial1

HardwareSerial Serial1(PA_10, PA_9);
HardwareSerial Serial3(PD_9, PD_8);

ELM327 elm;
void setup() {
  OBD_SERIAL.begin(38400);  // Set baud rate for OBD communication
  SERIAL_PORT.begin(9600);  // Set baud rate for output
  
  pinMode(PE5, OUTPUT);
    pinMode(PC2, OUTPUT);

  if (!elm.begin(OBD_SERIAL)) {
    SERIAL_PORT.println("Failed to connect to ELM327");
    digitalWrite(PE5, 1);
    delay(100);
    digitalWrite(PE5, 0);
    delay(100);
    while (1)
      ;
  }
  for (int i = 0; i < 20; i++) {
    digitalWrite(PE5, 1);
    delay(100);
    digitalWrite(PE5, 0);
    delay(100);
  }
  SERIAL_PORT.println("Connected to ELM327");
}

void loop() {
  float rpm = elm.rpm();
  float speed = elm.kph();
  SERIAL_PORT.print("Engine RPM: ");
  SERIAL_PORT.print(rpm);
  SERIAL_PORT.print("  VEHICLE_SPEED: ");
  SERIAL_PORT.println(speed);
      digitalWrite(PC2, 1);
    delay(100);
    digitalWrite(PC2, 0);
    delay(100);



  delay(3000);
}
