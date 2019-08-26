// RADIO and SD and LowPower (Arduino Sleeping) WOrking together

#include <LowPower.h>
#include "TinyGPS++.h"

#include <SPI.h>
#include <RFM69.h>
#include <SdFat.h>
#include <SoftwareSerial.h>


#define RADIO_LEVEL 4 // sets the radio signal strength during transmit
#define NETWORKID     0   // Must be the same for all nodes (0 to 255)
#define MYNODEID      1   // My node ID (0 to 255)
#define TONODEID      255 // Destination node ID (0 to 254, 255 = broadcast)
#define FREQUENCY RF69_915MHZ //set frequency of the radio (United States is 915MHZ, use 433MHZ for Madagascar

#define ARDUINO_GPS_RX 5  // GPS TX, Arduino RX pin
#define ARDUINO_GPS_TX 6  // GPS RX, Arduino TX pin


struct lemur_data{
  uint8_t node = MYNODEID;
  uint32_t date_raw = 0;
  uint32_t time_raw = 0;
};

SdFat sd;
SdFile myFile;
const int chipSelect = 9;
RFM69 radio;
SoftwareSerial ssGPS(ARDUINO_GPS_RX, ARDUINO_GPS_TX); // Create a SoftwareSerial
TinyGPSPlus tinyGPS; // Create a TinyGPSPlus object

int count = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // dtimeebug serial to usb
  ssGPS.begin(9600); // start GPS

  radio.initialize(FREQUENCY, MYNODEID, NETWORKID);
  //radio.setPowerLevel(RADIO_LEVEL); // Play around with this value
  radio.setHighPower();
  Serial.println("Arduino started!");

}

void loop() {
  ssGPS.begin(9600); // start GPS
  Serial.println(F("Collar Awake!"));
  Serial.print(F("Current Date: "));
  Serial.print(tinyGPS.date.value());
  Serial.println(F(""));
  Serial.print(F("Current Time: "));
  Serial.print(tinyGPS.time.value());
  Serial.println(F(""));
  
  if (!sd.begin(chipSelect, SPI_HALF_SPEED)) sd.initErrorHalt();
  if (!myFile.open("test.txt", O_RDWR | O_CREAT | O_AT_END)) {
    sd.errorHalt("opening test.txt for write failed");
  }
  myFile.println("testing 1, 2, 3.");
  myFile.close();
  Serial.println("done.");

  // From the documentation and online sources indicates that when the radio is set to send,
  // it should automatically set back to transmit mode
  Serial.println("pinging");
  if(ssGPS.available() > 0 ) {
     tinyGPS.encode(ssGPS.read()); // Send it to the encode function 
  }

  ping(); // sends data to the node
  Serial.flush(); // make sure the data sent is done before making things go to sleep.
  ssGPS.end();
  radio.sleep();
  for(int x = 0; x < 1; x++){
      LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); 
  }

}

static void smartDelay(unsigned long ms){
  // smart delay to parse gps data while it is sleeping
  unsigned long start = millis();
  do{
    while (ssGPS.available())
      tinyGPS.encode(ssGPS.read()); // Send it to the encode function
  } while (millis() - start < ms);
}

void ping(){
  lemur_data data;
  data.node = MYNODEID;
  data.date_raw = (uint32_t)millis()/1000;
  data.time_raw = count;
  radio.send(255,  (const void*)(&data) , sizeof(data));
  count++;
}

