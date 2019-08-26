/*
 * Lucas Mun 
 * 1/2018 - 5/2019
 * lucasmun@vt.edu
 * 
 * Lemur Collar Implementation
 * Please refer back to the associated document to see what works
 * and what does this current version of the code do.
*/

// Needed library includes
#include <SdFat.h> // SD card library
#include <SPI.h>  // SPI library
#include <RFM69.h> // Radio library
#include <LowPower.h> // Sleep mode configuration library
#include "TinyGPS++.h" // GPS NMEA decoder 
#include "mpu6050.h"   // Accelerometer Library
#include <SoftwareSerial.h> // Software serial
#include <Wire.h> // I2C library


#define MYNODEID      1   // My node ID (0 to 255)

#define STR(a) XSTR(a)    // Convert random stuff into Strings
#define XSTR(a) #a        // during compile time

#define ARDUINO_GPS_RX 5  // GPS TX, Arduino RX pin
#define ARDUINO_GPS_TX 6  // GPS RX, Arduino TX pin

#define NETWORKID     0   // Must be the same for all nodes for the nodes to be communicating (0 to 255)
#define TONODEID      255 // Destination node ID (0 to 254, 255 = broadcast)
#define FREQUENCY RF69_915MHZ //set frequency of the radio (United States is 915MHZ, use 433MHZ for Madagascar

#define SD_CS           9     // SD card chip select
#define RADIO_CS        10     // Radio chip select
#define BAUD           9600   // global baurdrate

#define POWER_EN  8  // enables gps and radio
#define RADIO_LEVEL 4 // sets the radio signal strength during transmit
#define maj             "0" // current version
#define mnr             "2"

#define GPS_INTERVAL        300 // 300 seconds    
#define ACCEL_INTERVAL        300 // 300 seconds (unused)   


// lemur struct data for ease of sending and saving data.
struct lemur_data{
  uint8_t node = MYNODEID;
  uint32_t date_raw = 0;
  uint32_t time_raw = 0;
};

RFM69 radio;
TinyGPSPlus tinyGPS; // Create a TinyGPSPlus object
SoftwareSerial ssGPS(ARDUINO_GPS_RX, ARDUINO_GPS_TX); // Create a SoftwareSerial
// MPU-6050 globals, NOTE: error and rest of the x_accl can not made globals
accel_t_gyro_union accel_t_gyro; // Union for accelerometer data (needed)
int error; // error flag for the accelerometer (needed)

SdFat sd; // SD card object
SdFile myFile; // File object








// Parses the struct parameter and unpacks it into a string
String parse_lemur_data(lemur_data *fetched){
  return String(fetched->date_raw) + "," + String(fetched->time_raw) + "," + String(fetched->node) + "," + String(radio.RSSI); 
}

// Sets up a data packet and sends over the radio
void ping(){
  lemur_data data;
  data.node = MYNODEID;
  data.date_raw = tinyGPS.date.value();
  data.time_raw = tinyGPS.time.value();
  radio.send(255,  (const void*)(&data) , sizeof(data));
}

// Logs the incoming string data and saves it to "data.csv"
void log_data(String file_name, String data){
  if (!myFile.open((char*)file_name.c_str(), O_RDWR | O_CREAT | O_AT_END)) {
    sd.errorHalt("opening file and write failed");
  }
  Serial.println(data);
  myFile.println(data);
  myFile.close(); // close file after finishing writing so SD card can go to sleep mode
}

// Logs the current snapshot of all of the peripherals (Date, Time, Location, Accel data) 
void log_curr_data(){
  if (!myFile.open("data.csv", O_RDWR | O_CREAT | O_AT_END)) {
    sd.errorHalt(F("opening file and write failed"));
  }
  
  myFile.print(tinyGPS.date.day()) + "/" + String(tinyGPS.date.month()) + "/" + String(tinyGPS.date.year());
  myFile.print(F(","));
  myFile.print(tinyGPS.time.hour()) + ":" + String(tinyGPS.time.minute()) + ":" + String(tinyGPS.time.second());
  myFile.print(F(","));
  myFile.print(tinyGPS.location.lat());
  myFile.print(F(","));
  myFile.print(tinyGPS.location.lng());
  myFile.print(F(","));
  myFile.print(tinyGPS.altitude.feet());
  myFile.print(F(","));
  myFile.print(tinyGPS.satellites.value());
  myFile.print(F(","));

  float x_acc, y_acc, z_acc, total_acc;
  x_acc = accel_t_gyro.value.x_accel; 
  y_acc = accel_t_gyro.value.y_accel;
  z_acc = accel_t_gyro.value.z_accel;
  total_acc = sqrt(x_acc*x_acc + y_acc*y_acc + z_acc*z_acc);
  myFile.print(total_acc,4);
  
  myFile.print("\n");
  myFile.close();
}

// Prints current snapshot data of the peripherals
void print_curr_data(){
  Serial.print(currentDate());
  Serial.print(F(","));
  Serial.print(currentTime());
  Serial.print(F(","));
  Serial.print(current_GPS());
  Serial.print(F(","));

  fixed_read(); // read mpu6050
  float x_acc, y_acc, z_acc, total_acc;
  x_acc = accel_t_gyro.value.x_accel; 
  y_acc = accel_t_gyro.value.y_accel;
  z_acc = accel_t_gyro.value.z_accel;
  total_acc = sqrt(x_acc*x_acc + y_acc*y_acc + z_acc*z_acc);
  Serial.print(total_acc,4);
  
  Serial.print("\n");
}

// Initializes the SD card and checks for any faults
int init_sd(){
  if (!sd.begin(SD_CS, SPI_HALF_SPEED)) sd.initErrorHalt();
  return 1;
}


// Returns the magnitude of the accelerometer data
float current_mpu(){
    String to_return = "";
    float x_acc, y_acc, z_acc, total_acc;

    fixed_read(); // read mpu6050
    x_acc = accel_t_gyro.value.x_accel; 
    y_acc = accel_t_gyro.value.y_accel;
    z_acc = accel_t_gyro.value.z_accel;
    total_acc = sqrt(x_acc*x_acc + y_acc*y_acc + z_acc*z_acc);
    return total_acc;
}

// Initializes the accelerometer 
void start_mpu6050(){
  // Starts MP6050 with correct starting parameters
  uint8_t c;
  error = MPU6050_read (MPU6050_PWR_MGMT_1, &c, 1); 
  MPU6050_write_reg (MPU6050_PWR_MGMT_1, 0);
}

// Fixes the byte positions within the accelerometer data
int fixed_read(){
  uint8_t swap;
  error = MPU6050_read (MPU6050_ACCEL_XOUT_H, (uint8_t *) &accel_t_gyro, sizeof(accel_t_gyro));
  SWAP (accel_t_gyro.reg.x_accel_h, accel_t_gyro.reg.x_accel_l);
  SWAP (accel_t_gyro.reg.y_accel_h, accel_t_gyro.reg.y_accel_l);
  SWAP (accel_t_gyro.reg.z_accel_h, accel_t_gyro.reg.z_accel_l);
  SWAP (accel_t_gyro.reg.t_h, accel_t_gyro.reg.t_l);
  SWAP (accel_t_gyro.reg.x_gyro_h, accel_t_gyro.reg.x_gyro_l);
  SWAP (accel_t_gyro.reg.y_gyro_h, accel_t_gyro.reg.y_gyro_l);
  SWAP (accel_t_gyro.reg.z_gyro_h, accel_t_gyro.reg.z_gyro_l);
}

// Delay function that allows polling (updating) of GPS locations
static void smartDelay(unsigned long ms){
  // smart delay to parse gps data while it is sleeping
  unsigned long start = millis();
  do{
    while (ssGPS.available())
      tinyGPS.encode(ssGPS.read()); // Send it to the encode function
  } while (millis() - start < ms);
}

// Returns the current GPS location as a string
String current_GPS(){
  // Returns latitude, longtitude, altitude, and amount of satilites in csv format
  // TODO: Don't convert to string
  String lati = String(tinyGPS.location.lat());
  String lngt = String(tinyGPS.location.lng());
  String alt =  String(tinyGPS.altitude.feet());
  String sat =  String(tinyGPS.satellites.value());
  return lati + "," + lngt + "," + alt + "," + sat;

}

// Returns current date as string DD/MM/YYYY
String currentDate(){
  String x = String(tinyGPS.date.day()) + "/" + String(tinyGPS.date.month()) + "/" + String(tinyGPS.date.year());
  return x;
}

// Returns current time as string HH:MM:SS
String currentTime(){
  String x = String(tinyGPS.time.hour()) + ":" + String(tinyGPS.time.minute()) + ":" + String(tinyGPS.time.second());
  return x;
}

// Experimental sleeping procedure where it sleeps for the duration of minute parameter
void sleep_for(int minutes){
  for(int x = 0; x < minutes; x++){ // this will happen for 4 minutes
    for(int y = 0; y < 60; y++){ // this will happen for 1 minute
      LowPower.powerDown(SLEEP_1S , ADC_OFF, BOD_OFF); 
    }
  }
  delay(100); // just make sure it finishes waking up?
}





































void setup() {
  Serial.begin(BAUD); // initalize serial to usb
  Wire.begin();      // enable i2c for mpu6050
  start_mpu6050();   // start mpu6050
  init_sd();         // start SD
    
  Serial.println(F("Version: " maj "." mnr));
  Serial.println(F("Current Node ID: " STR(MYNODEID)));
  radio.initialize(FREQUENCY, MYNODEID, NETWORKID); // Initialize radio based on pre-defined parameters
  radio.setPowerLevel(RADIO_LEVEL); // Play around with this value to get optimal radio strength, range: 0-31
  ssGPS.begin(BAUD); // start GPS

  // Initial read from GPS
  if(ssGPS.available()){
    tinyGPS.encode(ssGPS.read()); // Send it to the encode function
  }
  
  // Don't progress until lock with GPS is available
  // Go through forced routine if no fix is available. 
  // No point to start pinging if no location lock
  // NOTE: For testing purposes, the GPS goes into an infinite loop when there is no lock
  // Change this to "If no lock, proceeded to use last known location"
  int start = millis();
  uint8_t locked = 0;
  if(!(tinyGPS.location.lat() || tinyGPS.location.lng())){ // make sure lat and lng are not the same
    Serial.println("GPS No lock during setup"); // This routine makes sure GPS is locked before proceeding for testing
    Serial.println("Forced lock initiated");
    int cur_milis = millis();
    while(cur_milis - start < 180000){ // keep looking for 5 minutes
      Serial.println(F("Attempting to lock"));
      smartDelay(5000); 
      Serial.print(F("Number of Satelites: "));
      Serial.print(tinyGPS.satellites.value());
      Serial.print("\n");
      //print_curr_data();                                                                  //modified
      Serial.println(currentDate());
      Serial.println(tinyGPS.location.lat(), 6);
      Serial.println(tinyGPS.location.lng(), 6);
      
      if(tinyGPS.location.lat() || tinyGPS.location.lng()){ // GPS lock only if lat and long are not 0
        Serial.print(F("GPS locked\n"));
        locked = 1;
        break;
      }
      else{
        Serial.println(F("GPS failed to lock"));
      }
      int cur_milis = millis();
    }
    if(!locked){
      Serial.println("GPS Lock Failed. Halting");
      for(;;);
    }
  }
  else{
    Serial.println("GPS locked. Proceeding...");
  }
  
  // Ensures that it gets its first fix before it writes anything to the data.
  log_data(F("data.csv"),F("Date,Time,Lat,Long,Altitude,Satelites,Accel")); // Writes the header for CSV file
  log_data(F("contact.csv"),F("Date,Time,Node"));
  Serial.println("Log_data finished");
  log_curr_data(); // Logs current data if GPS lock as been done
  Serial.println("Initialization finished");
}




















void loop() { 
  ssGPS.begin(BAUD); // restart GPS
  // turn on the power supply. The Radio needs to be reinitallized every time it resets
  // just always keep rewriting this in case something happens. This ensures that
  // every new iteration will keep the radio and the GPS ON.

  if(ssGPS.available()){ // Attempts to read GPS
    tinyGPS.encode(ssGPS.read());
  }
      
  Serial.println(F("Collar Awake!"));
  Serial.print(F("Current Date: "));
  Serial.print(tinyGPS.date.value());
  Serial.println(F(""));
  Serial.print(F("Current Time: "));
  Serial.print(tinyGPS.time.value());
  Serial.println(F(""));

  Serial.println("Pinging 5 times");
  radio.receiveDone(); // this is needed wake up the radio after it is set asleep
  smartDelay(100); // poll the gps data (it is a delay but its the "lazy" way of doing it)
  if(tinyGPS.time.minute() % GPS_INTERVAL == 0){ // if the current minute is GPS_INTERVAL'th minute of the hour, or 15th minute of the hour
    for(int x = 0; x < 5; x++){
        ping();
        if (radio.receiveDone()){
            if (radio.DATALEN == sizeof(lemur_data)) {
              lemur_data *fetched = (lemur_data *)(radio.DATA); // fetches the data from the radio
              String data = parse_lemur_data(fetched); // unpack
              Serial.println(F("Lemur found!")); // notify the user
               log_data("contact.csv", data);  // logging operation (disabled for testing)
               //log_data_raw(*fetched);          // logging operation (disabled for testing) 
              smartDelay(100);  
            }
        }
        Serial.println(x);
        smartDelay(1000);
      }
  }

  // After it finishes pinging -> Just actively listen (currently set to 3 seconds)
  unsigned long start = millis();
  Serial.println("Actively Listening for 3 seconds");
  do{
      if (radio.receiveDone()){
          if (radio.DATALEN == sizeof(lemur_data)) {
            lemur_data *fetched = (lemur_data *)(radio.DATA); // fetches the data from the radio
            String data = parse_lemur_data(fetched); // unpack
            Serial.println(F("Lemur found!")); // notify the user
            log_data("contact.csv", data);  // logging operation (disabled for testing)
            //log_data_raw(*fetched);          // logging operation (disabled for testing) 
            smartDelay(100);  
          }
      }
  } while(millis() - start < 3000); // keep listening for 3 seconds. 
  
  log_curr_data(); // log current data
  Serial.println(F("Current data logged"));
  Serial.println("Going to sleep...");

 // This is the sleep routine
 // Make sure that the Serial (hardware and software serials) are flushed then end().
 // Otherwise, garbled messages will come out at the end if the buffer is not cleared
  radio.sleep();
  Serial.flush();
  ssGPS.flush();
  ssGPS.end();

}
