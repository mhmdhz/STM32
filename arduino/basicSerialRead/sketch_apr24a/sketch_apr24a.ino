#include <HardwareSerial.h>

// Define serial ports for OBD and output
#define SerialAT Serial2
HardwareSerial Serial2(PD_9, PD_8);  // Connected to SIM808 TX2,RX2
#define SERIAL_PORT Serial5
HardwareSerial Serial5(PD_2, PC_12);  //Connected to TTL to USB for serial debugging  TX3,RX3

#define LED PE5

#define SIM808_POWERKEY PC10
char data;

void setup() {
  // Initialize OBD serial port (adjust baud rate if needed)
  SerialAT.begin(115200);
  // digitalWrite(SIM808_POWERKEY, 1);

  delay(1000);

  // Initialize output serial port
  SERIAL_PORT.begin(115200);
  // pinMode(SIM808_POWERKEY, OUTPUT);
  // // put your setup code here, to run once:
  // pinMode(LED, OUTPUT);
  // // digitalWrite(LED, 0);

  // digitalWrite(SIM808_POWERKEY, 0);
  // delay(1000);
  // digitalWrite(SIM808_POWERKEY, 1);
  // delay(1000);
  // digitalWrite(SIM808_POWERKEY, 0);


  // Send a simple AT command to test connection with OBD-II device
  // SerialAT.println("AT+COPS=0");
  SERIAL_PORT.println("started !");

  // Wait a brief moment for potential response
  delay(1000);
}

void loop() {
  // Check if there's data available from the Serial Monitor
  // if (SERIAL_PORT.available()) {

  //   // Read the line from the Serial Monitor
  //   String line = SERIAL_PORT.readStringUntil('\n');

  //   // Write the line to the SerialAT port
  //   SerialAT.println(line);
  // }

  // Check if there's data available from the SerialAT port
  // if (SerialAT.available()) {
    SerialAT.println("{\"RequestData\":true}");
    // Read and print the line from SerialAT to SERIAL_PORT
    String line = SerialAT.readStringUntil('\n');
    SERIAL_PORT.println(line);
    delay(5000);
  // }
}
