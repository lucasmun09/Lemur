// Testing stability of radio communication

#include <RFM69.h>
#include <SPI.h>

#define NETWORKID     0   // Must be the same for all nodes (0 to 255)
#define MYNODEID      2   // My node ID (0 to 255)
#define TONODEID      255   // Destination node ID (0 to 254, 255 = broadcast)

#define FREQUENCY     RF69_915MHZ

#define USEACK        false // Request ACKs or not

#define LED           9 // LED positive pin // Packet sent/received indicator LED (optional):
#define GND           8 // LED ground pin

RFM69 radio;
struct lemur_data{
  uint8_t node = MYNODEID;
  uint32_t date_raw = 34340;    // gps.date.value()
  uint32_t time_raw = 54530;    // gps.time.value()
};


void setup()
{
  // Open a serial port so we can send keystrokes to the module:
  
  Serial.begin(9600);
  Serial.print("Node ");
  Serial.print(MYNODEID,DEC);
  Serial.println(" ready");  

  radio.initialize(FREQUENCY, MYNODEID, NETWORKID);  // Initialize the RFM69HCW:
  radio.setHighPower(); // Always use this for RFM69HCW

}

void loop()
{
  // static char sendbuffer[62];
  lemur_data cur_data;

  if (radio.receiveDone()) // Got one!
  {
    
    Serial.print("received from node ");
    Serial.print(radio.SENDERID, DEC);
    Serial.print(": [");

    // The actual message is contained in the DATA array,
    // and is DATALEN bytes in size:
    if (radio.DATALEN != sizeof(lemur_data)) {
      
      return;
    }

    lemur_data *fetched = (lemur_data *)(radio.DATA);
    Serial.println(fetched->node);
    Serial.println(fetched->date_raw);
    Serial.println(fetched->time_raw);
    Serial.print("], RSSI ");
    Serial.println(radio.RSSI);
    
    if (radio.ACKRequested())
    {
      radio.sendACK();
      Serial.println("ACK sent");
    }
  }
  delay(10);
}

