// Another Base station code that checks sending and receiving data from a transmitting radio
#include <RFM69.h>
#include <SPI.h>

#define NETWORKID     0   // Must be the same for all nodes (0 to 255)
#define MYNODEID      2   // Base is always 255
#define TONODEID      255   // Destination node ID (0 to 254, 255 = broadcast)

#define FREQUENCY     RF69_915MHZ


// CS is @ pin D10 by default

RFM69 radio;
struct lemur_data{
  uint8_t node = MYNODEID;
  uint32_t date_raw = 0;    // gps.date.value()
  uint32_t time_raw = 0;    // gps.time.value()
};


void setup(){
  // Open a serial port so we can send keystrokes to the module:
  
  Serial.begin(9600);
  Serial.println("Base Ready");
  Serial.print("Base address: ");
  Serial.print(MYNODEID,DEC);
  Serial.println(F(""));

  // no encryption with no ack due to the nature of broadcasting mode
  radio.initialize(FREQUENCY, MYNODEID, NETWORKID);  // Initialize the RFM69HCW:
  radio.setHighPower(); // Always use this for RFM69HCW
  Serial.println("Incoming ping example");
  Serial.println("-----------------------------");
  print_data();
  Serial.println("-----------------------------");
}

void loop()
{
  delay(1000);
  lemur_data cur_data;
  
  if (radio.receiveDone()){
    // make sure the length of the data is size of a normal lemur_data struct
    if (radio.DATALEN != sizeof(lemur_data)) {
      return;
    }
    print_data();
  }
}

void print_data(){
    lemur_data *fetched = (lemur_data *)(radio.DATA);
    Serial.print(F("Lemur "));
    Serial.print(radio.SENDERID, DEC);
    Serial.println(F(" found"));
    Serial.print("Lemur Data: ");
    Serial.print(fetched->node);
    Serial.print(F(", "));
    Serial.print(fetched->date_raw);
    Serial.print(F(", "));
    Serial.print(fetched->time_raw);
    Serial.print(F("\nRSSI:"));
    Serial.print(radio.RSSI);
    Serial.println(F(""));
}

