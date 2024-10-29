#include <SoftwareSerial.h>

#define ELM_RX_PIN 10  // Arduino RX (connects to ELM327 TX)
#define ELM_TX_PIN 11  // Arduino TX (connects to ELM327 RX

SoftwareSerial elmSerial(ELM_RX_PIN, ELM_TX_PIN); // RX, TX

void setup() {
  Serial.begin(115200);
  elmSerial.begin(38400); // Adjust baud rate if necessary

  Serial.println("Sending AT commands to ELM327...");
  delay(1000); // Wait for the module to initialize

  // Test communication
  sendATCommand("ATZ"); // Reset ELM327
  delay(2000);

  sendATCommand("ATE0"); // Disable echo
  delay(500);

  sendATCommand("ATL0"); // Disable linefeeds
  delay(500);

  sendATCommand("ATH0"); // Disable headers
}

void loop() {
  if (elmSerial.available()) {
    String response = elmSerial.readStringUntil('\n');
    Serial.print("ELM327: ");
    Serial.println(response);
  }

  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    elmSerial.println(command);
    Serial.print("Sent: ");
    Serial.println(command);
  }
}

void sendATCommand(String command) {
  Serial.print("Sending: ");
  Serial.println(command);
  elmSerial.println(command);
}
