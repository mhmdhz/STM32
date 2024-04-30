
#include <HardwareSerial.h>

// Define serial ports for OBD and output
#define OBD_SERIAL Serial1
HardwareSerial Serial1(PA_10, PA_9);  // Assuming correct pin assignments for OBD communication
#define SERIAL_PORT Serial3
HardwareSerial Serial3(PD_9, PD_8);  // Assuming correct pin assignments for output
char data =  ' ';

void setup() {
  // Initialize OBD serial port (adjust baud rate if needed)
  OBD_SERIAL.begin(38400);

  // Initialize output serial port
  SERIAL_PORT.begin(9600);

  // Send a simple AT command to test connection with OBD-II device
  OBD_SERIAL.println("ATZ");

  // Wait a brief moment for potential response
  delay(1000);
}

void loop() {
  // Check if there's data available from OBD
  if (OBD_SERIAL.available()) {
    // Read and print any received data to the output serial port
     data = OBD_SERIAL.read();
  }
  SERIAL_PORT.print(data);
  delay(100);
}
