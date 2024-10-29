//This code is best for cheking the obdii connection and frimware and uart
#include "Arduino.h"
#include "ELMduino.h"

// Define pins for LED (adjust according to your hardware setup)
#define LED_PIN PE5  // Example pin; change as needed
#define SERIAL_OBD Serial1
// Initialize ELM327 object
ELM327 elm;

// Define serial ports
HardwareSerial SerialDebug(PD_2, PC_12);  // Connected to TTL to USB for serial debugging TX, RX
HardwareSerial Serial1(PA_10, PA_9);    // Connected to OBD2 car diagnostic PORT TX, RX

// Define baud rates
#define DEBUG_BAUD 115200
#define OBD_BAUD 38400

// Variables to store RPM and speed
float rpm = 0;
float speed = 0;



// Function prototypes
void indicateError();
void indicateSuccess();

void setup() {
  // Initialize LED pin
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);  // Ensure LED is off initially

  // Initialize serial ports
  SerialDebug.begin(DEBUG_BAUD);
  SERIAL_OBD.begin(OBD_BAUD);



  SerialDebug.println("*************** Cheetah System Started ***************");
  connectToELM327();


  // for (int i = 0; i < 10; i++) {  // Reduced blink count for brevity
  //   digitalWrite(LED_PIN, HIGH);
  //   delay(100);
  //   digitalWrite(LED_PIN, LOW);
  //   delay(100);
  // }
}

void loop() {

OBD2_DATA();

}

// Function to indicate error by blinking LED rapidly
void indicateError() {
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(50);
    digitalWrite(LED_PIN, LOW);
    delay(50);
  }
}

// Function to indicate successful connection by blinking LED slowly
void indicateSuccess() {
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
}

void connectToELM327() {

  if (elm.begin(SERIAL_OBD)) {
    SerialDebug.println("Connected to ELM327 successfully.");
    indicateSuccess();
  } else {
    SerialDebug.println("Failed to connect to ELM327.");
    indicateError();
  }
}
void OBD2_DATA(){

  SerialDebug.println(elm.connected);
  
  if (elm.connected) {
    rpm = elm.rpm();  // Get the RPM directly
    if (elm.nb_rx_state == ELM_SUCCESS) {
      SerialDebug.print("RPM: ");
      SerialDebug.println(rpm);
      indicateSuccess();
    } else {
      SerialDebug.println("Failed to read RPM.");
      indicateError();
    }
    delay(100);
    speed = elm.kph();  // Get the speed directly
    if (elm.nb_rx_state == ELM_SUCCESS) {
      SerialDebug.print("Speed: ");
      SerialDebug.print(speed);
      SerialDebug.println(" kph");
      indicateSuccess();
    } else {
      SerialDebug.println("Failed to read Speed.");
      indicateError();
    }
      delay(100);

    SerialDebug.print("RPM: ");
    SerialDebug.print(rpm);
    SerialDebug.print(" | Speed: ");
    SerialDebug.print(speed);
    SerialDebug.println(" kph");

  }

  else {
    SerialDebug.println("ELM327 not connected.");
    connectToELM327();
    indicateError();
  }
  delay(500);


}
