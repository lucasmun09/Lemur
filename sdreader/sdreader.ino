// This tests the SD card module (It Works!)
#include <SdFat.h>
#include <SPI.h>


struct lemur_data{
  uint8_t node = 1;
  uint32_t date_raw = 0;
  uint32_t time_raw = 0;
};



SdFat sd;
SdFile myFile;
const int chipSelect = 9;
void setup() {
  Serial.begin(9600);
  if (!sd.begin(chipSelect, SPI_HALF_SPEED)) sd.initErrorHalt();
  if (!myFile.open("test.txt", O_RDWR | O_CREAT | O_AT_END)) {
    sd.errorHalt("opening test.txt for write failed");
  }
  myFile.println("testing 1, 2, 3.");
  myFile.close();
  Serial.println("done.");


}

void loop() {
  // put your main code here, to run repeatedly:

}
