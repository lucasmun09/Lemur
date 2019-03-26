rd/**Radio Slave Select Pin*/
static const uint8_t RFM69_CS = 10;
/**Radio Interrupt Pin*/
static const uint8_t RFM69_INT = 11;
/**Radio Reset Pin*/
static const uint8_t RFM69_RST = 6;
/**GPS Enable Pin*/
static const uint8_t GPS_EN = 12;
/**GPS Pulse per Second Pin*/
static const uint8_t PPS_PIN = 5;
/**SD Slave Select Pin*/
static const uint8_t SD_CS = 4;
/**LED Pin*/
static const uint8_t LED = 13;
/**Thermistor Pin*/
static const uint8_t THERM = 14;


void setup_SD(){
  
}

void setup() {
  // setup Pins
  pinMode(GPS_EN, OUTPUT);
  pinMode(SD_CS, OUTPUT);
  pinMode(PPS_PIN, INPUT_PULLUP);
  pinMode(LED, OUTPUT); //LED for flasher
  pinMode(THERM, INPUT); //thermistor for the battery
  
  // setup Serial
  Serial.begin(9600);
  // setup Radio

  // setup GPS

  // setup accelerometer

  //setup SD Card
}

void loop() {
  // put your main code here, to run repeatedly:

}
