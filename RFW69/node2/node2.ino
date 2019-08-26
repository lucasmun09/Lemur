// Testing stability of radio communication

#include <RFM69.h>
#include <SPI.h>

#define NETWORKID     0   // Must be the same for all nodes (0 to 255)
#define MYNODEID      1   // 1
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
  radio.setPowerLevel(31); // Always use this for RFM69HCW
  
}

void loop()
{
  delay(3000);
  // static char sendbuffer[62];
  lemur_data data;
  data.date_raw = 6969;
  data.time_raw = 0420;

  // SENDING


  radio.send(TONODEID, (const void*)(&data) , sizeof(data));
  Serial.print(millis());
  Serial.print(" Pinging...\n");
}

