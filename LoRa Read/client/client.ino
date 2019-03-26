#include <SoftwareSerial.h>

#define ARDUINO_GPS_RX 5 // GPS TX, Arduino RX pin
#define ARDUINO_GPS_TX 6 // GPS RX, Arduino TX pin

SoftwareSerial ss(ARDUINO_GPS_RX, ARDUINO_GPS_TX); // Create a SoftwareSerial

void setup() {
  Serial.begin(9600);
  ss.begin(9600);
}

void loop() {
  if (ss.available()) {
    int inByte = ss.read();
    Serial.write(inByte);
  }
}
