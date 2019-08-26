// Receiver for Radio communication (It works!)
#include <SPI.h>
#include <RFM69.h>

#define RADIO_LEVEL 4 // sets the radio signal strength during transmit
#define NETWORKID     0   // Must be the same for all nodes (0 to 255)
#define MYNODEID      1   // My node ID (0 to 255)
#define TONODEID      255 // Destination node ID (0 to 254, 255 = broadcast)
#define FREQUENCY RF69_915MHZ //set frequency of the radio (United States is 915MHZ, use 433MHZ for Madagascar

RFM69 radio;
struct lemur_data{
  uint8_t node = MYNODEID;
  uint32_t date_raw = 0;
  uint32_t time_raw = 0;
};


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // dtimeebug serial to usb
  radio.initialize(FREQUENCY, MYNODEID, NETWORKID);
  //radio.setPowerLevel(RADIO_LEVEL); // Play around with this value
    radio.setHighPower();
  Serial.println("Receiver initialized");
}

void loop() {
  //radio.send(254, " " , 1);
  radio.receiveDone();
  delay(100);
  if (radio.receiveDone()){       
    if (radio.DATALEN == sizeof(lemur_data)) {
      lemur_data *fetched = (lemur_data *)(radio.DATA);
      String data = parse_lemur_data(fetched);
      Serial.print("Receiver millis:");
      Serial.print(millis());
      Serial.println("");
      Serial.print(data);
      Serial.println("");
    }
  }
  radio.sleep();
}

String parse_lemur_data(lemur_data *fetched){
  return String(fetched->date_raw) + "," + String(fetched->time_raw) + "," + String(fetched->node) + "," + String(radio.RSSI); 
}

void radio_wakeup(){
  radio.receiveDone();  
}


