// Transmitter for Radio communication (It works!)

#include <LowPower.h>

#include <SPI.h>
#include <RFM69.h>


#define RADIO_LEVEL 4 // sets the radio signal strength during transmit
#define NETWORKID     0   // Must be the same for all nodes (0 to 255)
#define MYNODEID      1   // My node ID (0 to 255)
#define TONODEID      255 // Destination node ID (0 to 254, 255 = broadcast)
#define FREQUENCY RF69_915MHZ //set frequency of the radio (United States is 915MHZ, use 433MHZ for Madagascar

struct lemur_data{
  uint8_t node = MYNODEID;
  uint32_t date_raw = 0;
  uint32_t time_raw = 0;
};


RFM69 radio;
int count = 0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // dtimeebug serial to usb
  radio.initialize(FREQUENCY, MYNODEID, NETWORKID);
  //radio.setPowerLevel(RADIO_LEVEL); // Play around with this value
  radio.setHighPower();
  Serial.println("Arduino started!");

}

void loop() {

  // From the documentation and online sources indicates that when the radio is set to send,
  // it should automatically set back to transmit mode
  Serial.println("pinging");
  ping(); // sends data to the node
  Serial.flush(); // make sure the data sent is done before making things go to sleep.

  radio.sleep();
  for(int x = 0; x < 5; x++){
      LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF); 
      delay(100); 
  }
}
void ping(){
  lemur_data data;
  data.node = MYNODEID;
  data.date_raw = (uint32_t)millis()/1000;
  data.time_raw = count;
  radio.send(255,  (const void*)(&data) , sizeof(data));
  count++;
}

