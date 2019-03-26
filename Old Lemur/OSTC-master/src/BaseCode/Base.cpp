#include <RadioHead.h>
#include <radio_config_Si4460.h>
#include <RHCRC.h>
#include <RHDatagram.h>
#include <RHEncryptedDriver.h>
#include <RHGenericDriver.h>
#include <RHGenericSPI.h>
#include <RHHardwareSPI.h>
#include <RHMesh.h>
#include <RHNRFSPIDriver.h>
#include <RHReliableDatagram.h>
#include <RHRouter.h>
#include <RHSoftwareSPI.h>
#include <RHSPIDriver.h>
#include <RHTcpProtocol.h>
#include <RH_ASK.h>
#include <RH_CC110.h>
#include <RH_E32.h>
#include <RH_MRF89.h>
#include <RH_NRF24.h>
#include <RH_NRF51.h>
#include <RH_NRF905.h>
#include <RH_RF22.h>
#include <RH_RF24.h>
#include <RH_RF69.h>
#include <RH_RF95.h>
#include <RH_Serial.h>
#include <RH_TCP.h>

/**
 * @file
 * @brief Program to run the tracking collar base station
 * 
 * This program runs an Arduino powered base station.
 * The base station communicates with tracking collars over radio.
 */

//These are libraries that add functionality to the collar.
#include <Arduino.h>
#include <Adafruit_GPS.h> //Library for the GPS
#include <SPI.h> //Lets the radio talk on SPI
#include "math.h" //extra math functions!
#include <Wire.h> //needed for I2C bus
#include <SdFat.h> //Library for accessing the SD card
#include <RTCZero.h> //Library for the RTC clock. 
#include "wiring_private.h" // pinPeripheral() function
#include <RH_RF69.h> //radio library
#include <RHReliableDatagram.h> //Radio manager
#include <climits> //Max integer size
#include "Base.h"

/** Last recorded latitude of collar*/
static float latitude = 0;
/** North or South from equator*/
static char ns = 'N';
/** Last recorded longitude of collar*/
static float longitude = 0;
/** East or West of P.M.*/
static char ew = 'E';
/** Last recorded altitude*/
static double altitu = 0; //altitude

/** A more friendly name for the GPS Serial*/
#define SerialGPS Serial1
/**Adafruit GPS driver instance
 * Connects using hardware serial port
 */
Adafruit_GPS GPS(&SerialGPS);

/**RTC*/
RTCZero rtc;

/**Bluetooth Serial Port*/
Uart SerialBT (&sercom1, 11, 10, SERCOM_RX_PAD_0, UART_TX_PAD_2);

/**Filesystem object*/
SdFat sd;

/**RF69 radio driver*/
RH_RF69 rf69(RFM69_CS, RFM69_INT);

/**Radio manager
 * @note The Base's address is always 0
 */
RHReliableDatagram manager(rf69, 0);

/** Buffer for received message*/
static uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];

//Declare functions
static bool getGPS(unsigned long waittime);
static void enableGPS();
static void shutdownGPS();
static bool takeNote(String input);
static unsigned long elapsedTime(unsigned long starttime);
static void broadcast(String in);
static void transmit(String in, uint8_t address);
static String receive(uint16_t waittime, uint8_t * from = nullptr);
static void setDate(String incomingString);
static void downloadData(uint8_t collarNumber);
static String generateTimestamp();
static bool error(String message, Stream * stream = nullptr);
static void setup_sd();

//Declared in UART
void SERCOM1_Handler() //Inturrupt handler for Bluetooth serial
{
    SerialBT.IrqHandler();
}

/** 
 * @brief Configures the Arduino and it's components.
 * 
 * This is run on startup to configure the Arduino and it's components.
 * Serial, radio, GPS, Bluetooth, and SD card modules are setup.
 */
void setup()
{
    ///////////////////////////////////////////////////////////////UART setup (for debugging)
    Serial.begin(9600); //start UART at  baud.

    //If in wait_for_serial mode, wait for the serial to receive data before setup is continued.
    if (WAIT_FOR_DEBUG)
    {
        while (!Serial);
    }

    //////////////////////////////////////////////////////////////rf69 Radio Setup
    pinMode(RFM69_RST, OUTPUT);
    digitalWrite(RFM69_RST, LOW);
    // Manual Reset
    // "Pin RESET should be pulled high for a hundred microseconds, and then released. The user should then wait for 5 ms before using the module."
    digitalWrite(RFM69_RST, HIGH);
    //This delay can be as small as 100 microseconds; delay() uses milliseconds, so delay(1) should be more than enough
    delay(10);
    digitalWrite(RFM69_RST, LOW);
    delay(10);
    //Continually attempt to initialize radio until success
    while (!rf69.init()) {
      Serial.println("init failed");
    };
    if (!manager.init())
        Serial.println("init failed");
    // Defaults after init are 2.402 GHz (channel 2), 2Mbps, 0dBm
    if (!rf69.setFrequency(RF69_FREQ)) {
      Serial.println("setFrequency failed");
    }
    
        
    rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

    // Optional? Potentially remove to save power
    // The encryption key has to be the same as the one in the server
    //uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
    //                 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08
    //              };
    //rf69.setEncryptionKey(key);

    ///////////////////////////////////////////////////////////////UART setup (for bluetooth)
    SerialBT.begin(9600);
    // Assign pins 10 & 11 SERCOM functionality
    pinPeripheral(10, PIO_SERCOM);
    pinPeripheral(11, PIO_SERCOM);

    ///////////////////////////////////////////////////////////////Pin Definitions
    pinMode(GPS_EN, OUTPUT);
    pinMode(SD_CS, OUTPUT);


    ///////////////////////////////////////////////////////////////GPS Setup
    GPS.begin(9600); //start the GPS
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //get RMC and GGA data from GPS
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
    delay(1000); //wait 1000ms for the gps to boot
    SerialGPS.println(PMTK_Q_RELEASE); //ask for GPS Firmware version

    //////////////////////////////////////////////////////////////SD Setup
    setup_sd();
    
    //////////////////////////////////////////////////////////////RTC Setup
    rtc.begin(); //start the RTC
}

/**
 * @brief Sets up and tests the SD card
 *
 * Attempts to open a file to confirm SD card and filesystem is functional.
 */
void setup_sd()
{
    //////////////////////////////////////////////////////////////SD Setup
    while (!sd.begin(SD_CS))
    {
        Serial.println("SD Card Error... retrying...");
        delay(1000);
    }
    SdFile file;
    while (!file.open("/", O_READ))
    {
        Serial.println("Cannot open root... retrying...");
        delay(1000);
    }
    file.close();
    Serial.println("SD setup successful");
}

/**
 * @brief The execution loop
 *
 *  This loop is executed continuously until the Arduino is powered off.
 *  It's main purpose is to listen for commands from the user over Bluetooth.
 */
void loop()
{
    bool overheatingNodes[NUMBER_OF_NODES] = { false };
    unsigned long starttime = millis(); //start timer
    while (elapsedTime(starttime) < 500)
    {
        uint8_t from;
        String out = receive(500, &from);
        if (out.startsWith("Overheat") && !overheatingNodes[from])
        {
            String warning = "WARNING: Node ";
            warning += from + " is overheating!"; 
            error(warning, &SerialBT);
            overheatingNodes[from] = true;
        }
    }
  
    String incomingString = ""; //Bluetooth incoming string.
    static bool canprint = true;
    if (SerialBT.available() > 0) //if we have data on bluetooth waiting
    {
        incomingString = SerialBT.readString();           // read the incoming byte:
        if (incomingString.startsWith("setdate"))
        {
            setDate(incomingString);
        }
        else if (incomingString.startsWith("gettime")) //print the time to bluetooth
        {
            String out = "The time is: " + String(rtc.getMonth(), DEC) + "/" + String(rtc.getDay(), DEC) + "/" + String(rtc.getYear(), DEC) + " " + String(rtc.getHours(), DEC) + ":" + String(rtc.getMinutes(), DEC) + ":" + String(rtc.getSeconds(), DEC) + " Fix: " + String(GPS.fix, DEC);
            SerialBT.println(out);
        }
        else if (incomingString.startsWith("getgps")) //update GPS on base for timestamps
        {
            SerialBT.println("Getting GPS. Module will be inactive for up to 60 seconds...");
            getGPS(60);
            SerialBT.println("GPS Finder finished.");
        }
        else if (incomingString.startsWith("strt")) //start talking to lemurs
        {
            SerialBT.println("Opening Comms...");
            broadcast("start"); //start, expect a one char response
            SerialBT.println("Reading responses...");
            String connectedset = ""; //generate string
            unsigned long starttime = millis(); //start timer
            unsigned long elapsed = 0; //elapsed time
            while (elapsed < 5000) //while it's less than 5 seconds from the start
            {
                String out = receive(5000); //receive for 5 seconds
                Serial.print("in:");
                Serial.println(out);

                if (out != "") //if we dont get a blank output
                {
                    connectedset = connectedset + out + " "; 
                }
                elapsed = elapsedTime(starttime);
            }
			if (connectedset != "") //if got something back
			{
				SerialBT.println("Connected to: " + connectedset);
			}
			else //got nothing
			{
				SerialBT.println("Didn't connect to anything... Are you in range?");
			}
        }
        else if (incomingString.startsWith("stp")) //Shutdown Nodes
        {
            //stop coms
            SerialBT.println("Disconnecting from all...");
            broadcast("stp"); //send the start command
            delay(3000); //wait a little
            broadcast("stp"); //once more to be safe
            SerialBT.println("Done! Any nodes not shut down will automatically timeout after 15 minutes."); //tell user it's done
        }
        else if (incomingString.startsWith("wpd")) //Wipe Data
        {
            //data comes in as dld_node number
            uint8_t nn = (uint8_t) incomingString.substring(4).toInt();
            SerialBT.print("Erasing all data on: ");
            SerialBT.println(nn);
            String nameit = "wpd"; //build the command
            transmit(nameit, nn); //send the start command
            SerialBT.println("Sent request. Receiving...");
            String conf = receive(5000);
            if (conf.equals("d"))
            {
                SerialBT.println("Done!"); //tell user it's done
            }
            else
            {
                SerialBT.println("No Response to confirm erase. Reconnect and try again.");
            }
        }
        else if (incomingString.startsWith("dld")) //DownLoad Data
        {
            //data comes in as dld_node number
            uint8_t slaveNodeNumber = (uint8_t) incomingString.substring(4).toInt();
            downloadData(slaveNodeNumber); //download data
            SerialBT.println("Done!"); //tell user it's done
        }
        else if (incomingString.startsWith("fndr")) //Finder Function
        {
            SerialBT.println("Telling nodes in the area to start transmitting for 30 seconds...");
            broadcast("fndr"); //tell the lemurs to start transmitting for 30 seconds

            SerialBT.println("Starting Finder in 5 seconds; Searching for Ambient 915MHz radio in the area... Type anything in to cancel...");
            delay(1000);
            SerialBT.println("4...");
            delay(1000);
            SerialBT.println("3...");
            delay(1000);
            SerialBT.println("2...");
            delay(1000);
            SerialBT.println("1...");
            delay(1000);

            while (!SerialBT.available()) //if we have data on bluetooth waiting
            {
                String power = receive(1000);
                int16_t rssi = rf69.lastRssi();
                SerialBT.print("[");
                SerialBT.print(rssi);
                SerialBT.print("] "); // print raw dB
                int dots = 30 + round(-30 * (rssi + 27) / (-91 + 27)); //27 is the lowest RSSI I can get. 91 is the lowest.
                for (int i = 0; i < dots; i++)
                {
                    SerialBT.print("=");
                }
                SerialBT.println();
            }
            String burner = SerialBT.readString(); //burns up whatevers in RX
            SerialBT.println("Finding stopped.");
        }
		else if (incomingString.startsWith("flsh")) //Flash Command
        {
            //data comes in as flsh_node number
            uint8_t slaveNodeNumber = (uint8_t) incomingString.substring(5).toInt();
			SerialBT.print("Flashing node number ");
			SerialBT.print(slaveNodeNumber);
			SerialBT.println(" for duration set in node.h (default 10 sec). Will not respond to commands until flashing completes.");
			
			//transmit flash request
			transmit(incomingString, slaveNodeNumber);
        }
	   	else if (incomingString.startsWith("extshut")) //Extended Shutdown
        {
            //data comes in as extshut_MMDDYY
            uint8_t mn = incomingString.substring(8, 10).toInt(); //get month
            uint8_t da = incomingString.substring(10, 12).toInt(); //get day
            uint8_t yr = incomingString.substring(12, 14).toInt(); //get year
			      SerialBT.println("Shutting down all connected nodes until: " + String(mn, DEC) + "/" + String(da, DEC) + "/20" + String(yr, DEC));
		      	SerialBT.println("These nodes will only respond to contact at noon GMT until the above date. Are you sure? y/n");
			      while (SerialBT.available() == 0) //if we have data on bluetooth waiting
			      {} //do nothing until we get a response on bluetooth
      			String yesnostring = SerialBT.readString();
                yesnostring.trim();
			      if (yesnostring.equalsIgnoreCase("y") || yesnostring.equalsIgnoreCase("yes"))
			      {
				      String nameit = "extshut_" + String(mn, DEC) + String(da, DEC) + String(yr, DEC); //build the command
				      broadcast(nameit); //send the start command
				      SerialBT.println("Message sent. See you in the future! If you contact these nodes at noon GMT, they will break out of the extended shutdown cycle."); //tell user it's done
			      }
			      else
			      {
				      SerialBT.println("Extended Shutdown cancelled"); //tell user it's canceled
			      }
        }
        else //we're writing a note
        {
            String timestamp = generateTimestamp(); //make a timestamp for the note
            String tosave = timestamp + "," + formatGPS(outputType, seperateDirections, latitude, ns, longitude, ew) + "," + incomingString;
            Serial.println(tosave);
            if (takeNote(tosave)) //if it was saved to the SD card...
            {
                SerialBT.println("Saved: " + timestamp + "--" + incomingString); //done!
            }
            else
            {
                SerialBT.write("...saving failed!\n"); //something is wrong!
            }
        }
    }
    if ((rtc.getMinutes() == 59) && (canprint == true)) //make sure we only print once
    {
        SerialBT.println("if your RTC is up to date, you can call into the nodes now with 'strt'.");
        canprint = false;
    }
    if (rtc.getMinutes() != 59) //we can print again
    {
        canprint = true;
    }
    
}

/**
 * @brief Sets the date manually from user input.
 * 
 * In the event that GPS is performing poorly, the time and date can be set by the user manually.
 * This parses the String from the user and sets the RTC clock.
 * 
 * @param[in] incomingString    The time/date String to be parsed.
 */
void setDate(String incomingString)
{
    //set date manually using form setdatemmddyyhhmmss
    int m = incomingString.substring(7, 9).toInt();
    int d = incomingString.substring(9, 11).toInt();
    int y = incomingString.substring(11, 13).toInt();
    int h = incomingString.substring(13, 15).toInt();
    int mi = incomingString.substring(15, 17).toInt();
    int s = incomingString.substring(17, 19).toInt();
    rtc.setDate(d, m, y);
    rtc.setTime(h, mi, s);
    String out = "The date is: " + String(rtc.getMonth(), DEC) + "/" + String(rtc.getDay(), DEC) + "/" + String(rtc.getYear(), DEC) + " " + String(rtc.getHours(), DEC) + ":" + String(rtc.getMinutes(), DEC) + ":" + String(rtc.getSeconds(), DEC);
    SerialBT.println(out);
}

/**
 * @brief Generates a timestamp based on the date and time.
 * 
 * @return The timestamp
 */
String generateTimestamp()
{
    String out = String(rtc.getMonth(), DEC) + "/" + String(rtc.getDay(), DEC) + "/" + String(rtc.getYear(), DEC) + " " + String(rtc.getHours(), DEC) + ":" + String(rtc.getMinutes(), DEC) + ":" + String(rtc.getSeconds(), DEC);
    return out;
}

/**
 * @brief Enables GPS
 * 
 * Sets the GPS EN pin to LOW to enable the GPS
 */
void enableGPS()
{
    digitalWrite(GPS_EN, LOW); //turn on GPS so it can start satallite aquisition
}

/**
 * @brief Attempts to connect to GPS and updates the RTC.
 * 
 * Repeatedly attempts to get a GPS fix until success or the waittime is exceeded.
 * If successful, the RTC clock is updated.
 * 
 * The GPS is automatically enabled and disabled to save power.
 * 
 * @param[in] waittime How long to wait for a GPS fix before giving up
 * 
 * @returns Whether the operation was successful
 */
bool getGPS(unsigned long waittime)
{
    unsigned long starttime = millis();
    unsigned long elapsed = 0;
    enableGPS();
    SerialBT.print("Trying to get gps...");
    while (elapsed < (waittime * 1000))
    {
        GPS.read();
        //    Serial.print(c);
        if (GPS.newNMEAreceived())
        {
            if (GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
            {
                if (GPS.fix) //if we have a GPS fix...
                {
                    SerialBT.println("got a fix!");

                    SerialBT.print("searching for signal...");
                    int pulse = 0;
                    while ((pulse == 0) && (elapsed < (waittime * 1000))) //hold on until we get a pulse on the PPS. This syncs up all the nodes to the GMT second
                    {
                        pulse = digitalRead(PPS_PIN);
                        elapsed = elapsedTime(starttime);
                    }
                    SerialBT.println("Done!");

                    SerialBT.println("Setting RTC...");
                    rtc.setTime(GPS.hour, GPS.minute, GPS.seconds);
                    rtc.setDate(GPS.day, GPS.month, GPS.year);
                    
                    String testtime = generateTimestamp();
                    SerialBT.print("Time: ");
                    SerialBT.println(testtime);

                    //set all variables to new values!
                    latitude = GPS.latitude;
                    ns = GPS.lat;
                    longitude = GPS.longitude;
                    ew = GPS.lon;
                    altitu = GPS.altitude;

                    shutdownGPS();
                    return true;
                }
            }
        }
        elapsed = elapsedTime(starttime);
    }
    SerialBT.println("failure.");
    shutdownGPS();
    return false; //we didn't get a fix in the allotted time.
}

/**
 * @brief Shutdown the GPS
 * 
 * The GPS is shutdown. This is done to conserve power.
 */
void shutdownGPS()
{
    digitalWrite(GPS_EN, HIGH); //turn off GPS to conserve power
}

/**
 * @brief Writes a note to the info file
 * 
 * Writes a note to the info file on the SD card
 * 
 * @param[in] input Note to be written to the file
 * 
 * @return Whether the operation was successful
 */
bool takeNote(String input)
{
    digitalWrite(SD_CS, HIGH);//Turn on SD CS pin
    SerialBT.println("Writing to info.csv");
    SdFile info;
    if (info.open("info.csv", O_CREAT | O_WRITE | O_APPEND)) //if the file opened
    {
        SerialBT.println("Success!");
        info.println(input);
        info.close();
        digitalWrite(SD_CS, LOW);//Turn off SD CS pin
        return true;
    }
    else
    {
        SerialBT.println("Failed :(");
        digitalWrite(SD_CS, LOW);//Turn off SD CS pin
        return false; // the SD didn't open
    }
}

/**
 * @brief Broadcasts a message over radio
 *
 * Send a message to all radios in range.
 * Because the message is sent to the broadcast address, no ACK responses are received.
 *
 * @param[in] in The string to be broadcast.
 */
void broadcast(String in)
{ 

    transmit(in,RH_BROADCAST_ADDRESS);
  
}

/**
 * @brief Sends a message over radio
 *
 * Send a message to the specified address
 *
 * @param[in] in The string to be sent.
 * @param[in] address The address to send the message to.
 */
void transmit(String in, uint8_t address)
{
  
    Serial.print("Transmitting: ");
    uint8_t data[RH_RF69_MAX_MESSAGE_LEN];
    in.getBytes(data, RH_RF69_MAX_MESSAGE_LEN);
    manager.sendtoWait(data, sizeof(data), address);
  
    //rf69.send(data,sizeof(data));
}

/**
 * @brief Listens for and returns a message from the radio.
 * 
 * Listens for a message for the specified waittime.
 * If a message is received, ACK (unless it is a broadcast) and return the message as a String.
 * Otherwise, if there is no message, return an empty String.
 * 
 * @param[in] waittime The amount of time to wait before giving up.
 * 
 * @return The message or an empty String if no message is received.
 */
String receive(uint16_t waittime, uint8_t * from)
{
    uint8_t len = sizeof(buf);
    //Serial.print("Receiving..."); Serial.println();
    if(manager.recvfromAckTimeout(buf, &len, waittime, from))
    {
        Serial.println("Received message");
        Serial.print("[RSSI: ");
        Serial.print(rf69.lastRssi());
        Serial.print("] ");
        Serial.print("Received: ");
        Serial.println((char*)buf);
        return (char*)buf;
    }
    //Serial.println("Message timed out");
    return "";
}

/**
 * @brief Returns the elapsed time given a start time.
 * 
 * Returns how much time has passed since the given start time. Due to overflow, becomes inaccurate after ~50 days
 * 
 * @param[in] starttime The time the timer was started.
 * @return The elapsed time since starttime
 */
unsigned long elapsedTime(unsigned long starttime)
{
    unsigned long currenttime = millis();
    if (currenttime >= starttime)
    {
        return (currenttime - starttime);
    }
    else
    {
        //we overflowed in the middle of this millis function! AHHHHH
        return (ULONG_MAX - starttime + currenttime + 1);
    }
}

/**
 * @brief Requests and downloads data from a tracking collar.
 * 
 * Requests GPS, accelerometer, and other tracking data from the specified collar.
 * Stores said data in a data file prepended with the collar's number.
 * 
 * @todo Make sure SD is disabled appropriately.
 * 
 * @param[in] collarNumber The number of the collar to request data from.
 */
void downloadData(uint8_t collarNumber)
{
    SerialBT.print("Downloading Data from: ");
    SerialBT.println(collarNumber);
    //Plus 1 because null characters
    uint8_t msg[] = "dld";
    if(!manager.sendtoWait(msg, sizeof(msg), collarNumber))
    {
        error("Could not connect to node for data transfer", &SerialBT);
        return;
    }
    
    //digitalWrite(SD_CS, HIGH);//Turn on SD CS pin
    String filename = String(collarNumber, DEC) + "_data.csv"; //make a file name
    SerialBT.print("Writing to: ");
    SerialBT.println(filename);
    SdFile data;
    if (data.open(filename.c_str(), O_CREAT | O_WRITE | O_APPEND))
    {
        SerialBT.println("File opened");
        uint32_t initialSize = data.fileSize();
        unsigned long startTime = millis();
        uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
        uint8_t len = sizeof(buf);
        
        bool received = true;
        
        while ((!SerialBT.available()) && (received = manager.recvfromAckTimeout(buf, &len, 1200), received))
        {
            //If the packet is marked as an end message
            if(manager.headerFlags() % 2 == 1)
            {
                break;
            }
            data.write(buf, len);
            len = sizeof(buf);
        }
        if(!received)
        {
            error("Lost connection to node", &SerialBT);
        }
        unsigned long endTime = millis();
        unsigned long deltaTime = endTime - startTime;
        uint32_t deltaSize = data.fileSize() - initialSize;
        SerialBT.print("Transmit size: ");
        SerialBT.println(deltaSize);
        SerialBT.print("Elapsed time: ");
        SerialBT.println((float)deltaTime/1000);
        SerialBT.print("Transmit rate: ");
        SerialBT.print(deltaSize * 1000 / deltaTime);
        SerialBT.println("b/s");
    }
    else
    {
        SerialBT.println("Failed to open SD!");
    }

    //To ensure the node gives up
    delay(1000);
    
    String burner = SerialBT.readString(); //burns up whatevers in RX
    data.close(); //close the file
    //digitalWrite(SD_CS, LOW);//Turn off SD CS pin
}

/**
 * @brief Prints error message to file and to serial output
 */
bool error(String message, Stream * stream)
{
    if(stream != nullptr)
    {
        stream->println(message);
    }
    SdFile error;
    if(error.open("error.txt", O_CREAT | O_WRITE | O_APPEND))
    {
        error.println(generateTimestamp() + " " + message);
        error.close();
        return true;
    }
    return false;
}
