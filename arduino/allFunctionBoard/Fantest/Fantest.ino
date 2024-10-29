#define Fan PE9

#define SERIAL_PORT Serial5
HardwareSerial Serial5(PD_2, PC_12);  //Connected to TTL to USB for serial debuging  TX3,RX3

void setup() {
  SERIAL_PORT.begin(115200);
    pinMode(Fan, OUTPUT);

  SERIAL_PORT.println("Started !");
}

void loop() {
  digitalWrite(Fan, 1);
  SERIAL_PORT.println("FAN ON !");

  delay(10000);
  // SERIAL_PORT.println("Started !");

  SERIAL_PORT.println("FAN OFF !");

  digitalWrite(Fan, 0);
  delay(10000);
}
