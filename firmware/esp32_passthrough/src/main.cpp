// ESP32 UART passthrough: USB-CDC (UART0 via CH340) <-> Servo bus (UART2)
//
// Pins are set via build flags in platformio.ini (SERVO_RX_PIN, SERVO_TX_PIN).
// Default 16/17 = Arduino Serial2 default. Adjust if servos don't respond.
//
// USB side runs at 1000000 baud (matches servo bus speed).
// Servo bus runs at 1000000 baud (FeeTech default).
//
// Flash this, then connect to /dev/ttyUSB0 at 1000000 baud and speak
// FeeTech protocol. The ESP32 just shuffles bytes between the two UARTs.

#include <Arduino.h>

#ifndef SERVO_RX_PIN
#define SERVO_RX_PIN 16
#endif
#ifndef SERVO_TX_PIN
#define SERVO_TX_PIN 17
#endif

#define BAUD 1000000

void setup() {
  Serial.begin(BAUD);                                              // USB side (UART0)
  Serial2.begin(BAUD, SERIAL_8N1, SERVO_RX_PIN, SERVO_TX_PIN);     // Servo bus (UART2)
}

void loop() {
  // PC -> servo bus
  while (Serial.available()) {
    Serial2.write(Serial.read());
  }
  // Servo bus -> PC
  while (Serial2.available()) {
    Serial.write(Serial2.read());
  }
}
