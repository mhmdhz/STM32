  
  
  #include "Arduino.h"
  #include "ELMduino.h"
  
  #define SERIAL_PORT Serial3
  #define OBD_SERIAL Serial1
  
  HardwareSerial Serial1(PA_10, PA_9);
  HardwareSerial SERIAL_PORT(PD_9, PD_8);
  
  
  
  ELM327 elm;
  
  void setup() {
    OBD_SERIAL.begin(38400);  // Set baud rate for OBD communication
    SERIAL_PORT.begin(9600);  // Set baud rate for output
    SERIAL_PORT.println(" ELM327");
    pinMode(PE5, OUTPUT);
    digitalWrite(PE5, 1);
    delay(100);
    // digitalWrite(PE5, 0);
    delay(100);
  
    // Attempt connection to ELM327 on OBD serial port
    if (!elm.begin(OBD_SERIAL)) {
      SERIAL_PORT.println("Failed to connect to ELM327");
      digitalWrite(PE5, 1);
      delay(100);
      digitalWrite(PE5, 0);
      delay(100);
      while (1)
        ;
    }
    for (int i = 0; i < 10; i++) {
      digitalWrite(PE5, 1);
      delay(100);
      digitalWrite(PE5, 0);
      delay(100);
    }
    SERIAL_PORT.println("Connected to ELM327");
  }
  
  void loop() {
    // Example: Read engine RPM (replace with desired PID)
    uint8_t pid = 0x0C;  // PID for RPM
  
    float rpm = elm.rpm();
    if (rpm < 0) {
      SERIAL_PORT.println("Failed to read RPM");
    } else {
      SERIAL_PORT.print("Engine RPM: ");
      SERIAL_PORT.println(rpm);
    }
  
    // Delay between readings (adjust as needed)
    delay(1000);
  }
