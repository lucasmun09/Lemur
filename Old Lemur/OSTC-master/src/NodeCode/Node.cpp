
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
 * @brief Program to run a tracking collar
 *
 * This program runs the tracking collar. It controls all tracking mechanisms such as GPS and accelerometers,
 * and communicates with a base station to transmit all collected data.
 *
 * This program is (intended to be) optimized for power consumption so it can run for long periods of time with little external power.
 */

//These are libraries that add functionality to the collar.
#include <Arduino.h>
#include <Adafruit_GPS.h> //Library for the GPS
#include <SPI.h> //Lets the radio talk on SPI
#include "math.h" //extra math functions!
#include <Adafruit_MMA8451.h> //library for the accelerometer
#include <Adafruit_Sensor.h> //Backend for the accelerometer
#include <Wire.h> //needed for I2C bus
#include <SdFat.h> //Library for accessing the SD card
#include <RTCZero.h> //Library for the RTC clock. 
#include <RH_RF69.h> //radio library
#include <RHReliableDatagram.h> //message system with ACK
#include <climits> //Max integer size
#include <cstdlib>
#include "Node.h" //Config

static int mn; //wakeup time for extended shutdown month
static int da; //wakeup time for extended shutdown day
static int yr; //wakeup time for extended shutdown year
static float latitude = 0.0;
static char ns = 'N'; //North or South?
static float longitude = 0.0;
static char ew = 'E'; //East or West?
static float hdop; //horizontal dilution of precision

static uint8_t nodenumber; //this node's ID number

static volatile bool continuePing;
static volatile bool ppsInt;

/** Buffer for received message*/
static uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];

extern "C" char *sbrk(int i); //free ram

//initialize instance of classes
Adafruit_MMA8451 mma = Adafruit_MMA8451(); //start Accel
/** A more friendly name for the GPS Serial*/
#define SerialGPS Serial1
/**Adafruit GPS driver instance
 * Connects using hardware serial port
 */
Adafruit_GPS GPS(&SerialGPS);

/**RTC*/
RTCZero rtc;

/**Filesystem object*/
SdFat sd;

/**RF69 radio driver
 * @todo SPI Frequency? (8MHz)
 */
RH_RF69 rf69(RFM69_CS, RFM69_INT);

/**Radio manager
 * @note The address is set later
 */
RHReliableDatagram manager(rf69, 1);

//Declare functions
static void setup_radio();
static void setup_sd();
static bool getGPS(unsigned long waittime);
static void handshake();
static void irq();
static void gpsShutdown();
static double getAccel();
static double getBatt();
static int isImportantTime();
static String dataformatter(const double accel, const uint8_t * const nearbyNodes, const uint8_t nodeCount, const double power,const int * const rssi_list);
static void writeToSD(String input);
static void wipeData();
static bool ping(uint8_t * const nodeList, uint8_t * const nodeCount,int * const rssi_list);
static void oopirq();
static void extendedShutdown();
static unsigned long elapsedTime(unsigned long starttime);
static void extendedAlarm();
static void listenForMaster(int timetowait, bool * wake = nullptr);
static void broadcast(String in);
static void transmit(String in, uint8_t address);
static String receive(uint16_t timeout, uint8_t * to);
static void setRadioLowPower();
static void setRadioHighPower();
static double avAccel();
static uint32_t freeRam();
static void uploadData();
static void restartradio();
static void turnRadioOn();
static void turnRadioOff();
static void turnSDOn();
static void turnSDOff();
static void outofPower();
static void enableGPS();
static void ping_IRQ();
static void default_timer();
static bool isSDOn();
static double checkBattTemp();
static void flash(unsigned int flashtime);
static String generateTimestamp();
static bool error(String message, Stream * stream = nullptr);

/**
 * @brief Configures the Arduino and it's components.
 *
 * This is run on startup to configure the Arduino and it's components.
 * Serial, radio, GPS, accelerometer, and SD card modules are setup.
 */
void setup()
{
    //UART setup (for debugging)
    if(SERIAL_ENABLE)
    {
        Serial.begin(9600);
        //If in wait_for_serial mode, wait for the serial to receive data before setup is continued.
        if(WAIT_FOR_DEBUG)
        {
            while (!Serial);
        }
    }

    ///////////////////////////////////////////////////////////////Pin Definitions
    pinMode(GPS_EN, OUTPUT);
    pinMode(SD_CS, OUTPUT);
    pinMode(PPS_PIN, INPUT_PULLUP);
    pinMode(LED, OUTPUT); //LED for flasher
    pinMode(THERM, INPUT); //thermistor for the battery

    setup_sd();

    //////////////////////////////////////////////////////////////Get Node Number
    SdFile file;
    while(!file.open("node.txt", O_READ));
    
    //Ugly hack to read nodenumber
    char nstr[4] = {0};
    file.read(nstr, 3);
    //Why doesn't C++ have a proper stous?
    nodenumber = (uint8_t) atoi(nstr);
    file.close();
    
    if(nodenumber == 0 || nodenumber == 255)
    {
      //  Serial.println("Invalid nodenumber");
        error("Invalid nodenumber", &Serial);
        for(;;);
    }
    
    Serial.print("Node number set to: ");
    Serial.println(nodenumber);

    if (RADIO_ENABLE)
    {
        Serial.println("Enabling radio");
        setup_radio();
    }

    ///////////////////////////////////////////////////////////////Accelerometer setup
    Serial.println("Enabling accelerometer");
    mma.begin(); //start the accelerometer
    if (accelRange == 0)
	{
		mma.setRange(MMA8451_RANGE_2_G); //set range to +/- 2g.
	}
	else if (accelRange == 2)
	{
		mma.setRange(MMA8451_RANGE_8_G); //set range to +/- 8g.
	}
	else
	{
		mma.setRange(MMA8451_RANGE_4_G); //set range to +/- 4g by default.
	}

    //////////////////////////////////////////////////////////////RTC Setup
    rtc.begin(); //start the RTC
    rtc.setTime(0, 0, 0);
    rtc.setDate(0, 0, 0);
    Serial.println("Time/date set to 0");

    if (GPS_ENABLE)
    {
        Serial.println("Enabling GPS");
        ///////////////////////////////////////////////////////////////GPS Setup
        GPS.begin(9600); //start the GPS
        GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //get RMC and GGA data from GPS
        GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
        delay(1000); //wait 1000ms for the gps to boot
        SerialGPS.println(PMTK_Q_RELEASE); //ask for GPS Firmware version

        if(!getGPS(60))//get GPS data to update the RTC. Will run for X seconds or until it gets a fix
        {
            //If we fail to get a fix, we'll use the best time the GPS could get
            rtc.setTime(GPS.hour, GPS.minute, GPS.seconds);
            rtc.setDate(GPS.day, GPS.month, GPS.year);
        }
    }
    gpsShutdown();
    digitalWrite(LED, LOW);

    default_timer();
    
    turnRadioOff();
    turnSDOff();
    Serial.println("End of setup");
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
    bool exists = file.exists("data.csv");
    file.close();
    while (!file.open("data.csv", O_CREAT | O_WRITE | O_APPEND))
    {
        Serial.println("Cannot open file... retrying...");
        delay(1000);
    }
    if(!exists)
    {
      //print headers
      if((outputType == 0) || (outputType == 1)) //using lat/long
      {
        if (seperateDirections)
        {
          file.println("Date,Time,Latitude,N/S,Longitude,E/W,HDOP,Acceleration,Battery Voltage,Nearby Nodes");
        }
        else //do not seperate directions
        {
          file.println("Date,Time,Latitude,Longitude,HDOP,Acceleration,Battery Voltage,Nearby Nodes");
        }
      }
      else //use UTM
      {
        if (seperateDirections)
        {
          file.println("Date,Time,UTM Zone,Zone Letter,Easting,Northing,HDOP,Acceleration,Battery Voltage,Nearby Nodes");
        }
        else //do not seperate directions
        {
          file.println("Date,Time,UTM Zone,Easting,Northing,HDOP,Acceleration,Battery Voltage,Nearby Nodes");
        }
      }
    }
    file.close(); //close the data file. we know it opens though.
    Serial.println("SD setup successful");
}

/**
 * @brief Sets up radio
 *
 * Enables radio, sets frequency and power, and starts the manager.
 */
void setup_radio()
{
    ////////////////////////////////////////////////////////////rf69 Radio Setup
    pinMode(RFM69_RST, OUTPUT);
    digitalWrite(RFM69_RST, LOW);

    //A delay of 10 ms is recommended before use.
    delay(10);
  
    while (!manager.init())
    {
        Serial.println("RadioServer Manager failed to start");
        restartradio();
    }
    
    if (!rf69.setFrequency(RF69_FREQ))
    {
        error("setFrequency failed", &Serial);
    }
    //rf69.setEncryptionKey(key);
    
    rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW
    //Sets radio address to node number
    manager.setThisAddress(nodenumber);
}

/**
 * @brief Activates the default timer
 * 
 * Sets the RTC to use the default, every second, timer.
 */
void default_timer()
{
    rtc.disableAlarm();
    rtc.detachInterrupt();
    rtc.setAlarmTime(0, 0, 59); //calls the update on the 59th second so the time is 00.
    rtc.enableAlarm(rtc.MATCH_SS); //Wake when the seconds match
    rtc.attachInterrupt(irq); //attach Inturrupt vector
}

/**
 * @brief The execution loop
 *
 *  This loop is executed once a minute using the RTC to put the Arduino in standby.
 *  It checks the time to see whether to perform the following functions:
 *      Ping other collars
 *      Update GPS
 *      Listen for base commands
 *
 *  Also writes current/last position, current acceleration, and time to a log file.
 */
void loop()
{
    if (!NO_WAIT)
    {
        rtc.standbyMode(); //Sleep until an inturrupt. The inturrupt is called every minute
    }
    
    if(RADIO_ENABLE)
    {
        turnRadioOn();
    }
    turnSDOn();
    
    //****************************************//

    Serial.println(generateTimestamp());
	
    double temperature = checkBattTemp(); //check the temperature of the battery
    if(temperature > safeTemp)
    {
      transmit("Overheat", 0);
      error("Overheat", &Serial);
    }
    
    double power = getBatt(); //get our battery voltage
    if (power < POWER_THRESH) //if we are less than our "lowest power" setting
    {
        //We're out of power! Stop doing everything else and shut down until we power up!
        outofPower();
    }

    int importance;
    if(DEBUG_IMPORTANCE)
    {
        importance = DEBUG_IMPORTANCE;
    }
    else
    {
        importance = isImportantTime(); //is this time important?
    }

    uint8_t nearbyNodes[NUMBER_OF_NODES];
    int nearbyStrength[NUMBER_OF_NODES];
    uint8_t nodeCount = 0;

    Serial.print("Free RAM: ");
    Serial.println(freeRam());
    if (importance == 1)
    {
        //time for pinging. takes half a second
        Serial.println("Pinging others...");
        ping(nearbyNodes, &nodeCount,nearbyStrength);
        Serial.print("Nearby: ");
        for(uint8_t i = 0; i < nodeCount; i++)
        {
            Serial.print(nearbyNodes[i]);Serial.print(" ");
        }
         for(uint8_t i = 0; i < nodeCount; i++)
        {
            Serial.print(nearbyStrength[i]);Serial.print(" ");
        }
        Serial.println();
    }
    else if (importance == 2)
    {
        //time for pinging. takes half a second
        Serial.println("Pinging others...");
        ping(nearbyNodes, &nodeCount,nearbyStrength);
        Serial.print("Nearby: ");
        for(uint8_t i = 0; i < nodeCount; i++)
        {
            Serial.print(nearbyNodes[i]);Serial.print(" ");
        }
        for(uint8_t i = 0; i < nodeCount; i++)
        {
            Serial.print(nearbyStrength[i]);Serial.print(" ");
        }
        Serial.println();
        //NOTE: If GPS is moved before Ping, the GPS will take longer for different nodes.
        //This makes that implementation a bit more complicated (but not so bad)
        //time for GPS/timer update
        Serial.print("Looking for GPS satallites...");
        getGPS(45); //takes maybe 45 seconds. inturrupted when finished
        Serial.println("Done!");
    }
    else if (importance == 3)
    {
        //time to listen to master
        Serial.println("Looking for master...");
        setRadioHighPower();
        listenForMaster(50000);
        setRadioLowPower();
        Serial.println("Done!");
    }

    Serial.print("Getting Acceleration...");
    double accelval = avAccel(); //get accleration data. Takes 1 second.
    Serial.println("Done!");

    String outputstring = dataformatter(accelval, nearbyNodes, nodeCount, power,nearbyStrength); //write timestamp, last/new location, and new acclerationn

  //  turnSDOn();
    writeToSD(outputstring); //write it
    turnSDOff();

    Serial.println("Resetting location variables...");
    //blank out variables to prevent aliasing
    latitude = 0;
    ew = ' '; //East or West?
    longitude = 0;
    ns = ' '; //North or South?
    hdop = 0; //horizontal dilution of precision

    if(RADIO_ENABLE)
    {
        turnRadioOff();
    }
}

/**
 * @brief Base station communication loop
 *
 * If the collar is contacted by the base station during the normal execution loop, it enters communication mode.
 *
 * This mode listens for commands from the base station, including:
 *      fndr: Alerts the base of its presence, base takes note of RSSI
 *      dld: Uploads all logged data to the base station.
 *      wpd: Wipes the collar of all data.
 *      extshut: Shuts down the node for an extended period of time.
 *      stp: Ends communication, returns to normal loop.
 */
void handshake()
{
    rtc.detachInterrupt(); //stop inturrupting.
    rtc.disableAlarm(); //turn off the Alarm so it doesnt inturrupt
    Serial.print("Inside Hanshake Function");
    Serial.println();
    //wait for stop command or timeout

    //listen for data

    //if download data command is called..
    //download data

    //if shutdown command is called
    //enable longterm shutdown

    //if finder
    //transmit node number for 30 seconds.

	//if flash
	//flash the LED if this node
	
    //if stop command
    //stop

    unsigned long overallstarttime = millis();
    bool stopslave = false;
    while (stopslave != true)
    {
        unsigned long overallelapsed = 0;
        uint8_t to = 0;
        String received = receive(1000, &to);
        if(received.length() > 0)
        {
          Serial.print("Received: ");
          Serial.println(received);
          //finder command
          if (received.startsWith("fndr"))
          {
              unsigned long starttime = millis();
              unsigned long elapsed = 0;
              //transmit for 30 seconds...
              while (elapsed < 30000)
              {
                  broadcast(String(nodenumber, DEC));
                  elapsed = elapsedTime(starttime);
              }
              overallelapsed = 0;
              overallstarttime = millis();
          }
          //dld command
          if (received.startsWith("dld"))
          {
              Serial.print("Received data transfer request");
              if (to == nodenumber)
              {
                  uploadData();
              }
              else
              {
                  Serial.print("dld should NOT be broadcast");
              }
              overallelapsed = 0;
              overallstarttime = millis();
          }
          //erase command
          if (received.startsWith("wpd"))
          {
              Serial.print("Received data transfer request");
              if (to == nodenumber)
              {
                  Serial.println("Wiping data...");
                  wipeData();
                  transmit("d", 0);
              }
              else
              {
                  Serial.print("wpd should NOT be broadcast");
              }
              overallelapsed = 0;
              overallstarttime = millis();
          }
          //flsh command
          if (received.startsWith("flsh"))
          {
              //data comes in a flsh_nn
              Serial.print("Received flash request.");
              if (to == nodenumber)
              {
                  flash(flashDuration); //flash the LED for flashDuration seconds
              }
              overallelapsed = 0; //reset overall timer
              overallstarttime = millis(); //set new overall timer
          }
          //extended shutdown command in form extshut_mmddyy
          if (received.startsWith("extshut"))
          {
              mn = received.substring(8, 10).toInt(); //get month
              da = received.substring(10, 12).toInt(); //get day
              yr = received.substring(12, 14).toInt(); //get year
              extendedShutdown(); //shut it down
          }
          //stop command
          if (received.startsWith("stp"))
          {
              stopslave = true; //we're done here
          }
        }

        overallelapsed = elapsedTime(overallstarttime); //take elapsed time
        if (overallelapsed > 300000) //if we are connected for more than 5 minutes without another command...
        {
            setRadioLowPower();
            stopslave = true; //timeout
        }
    }

    broadcast(" ");
    rtc.enableAlarm(rtc.MATCH_SS); //Wake when the seconds match
    rtc.attachInterrupt(irq); //attach Inturrupt vector
}

//Inturrupt vector for the RTC. Does not do anything, but keeps timing by its very existance.
void irq() 
{
	//nothing here
}

/**
 * @brief Enables GPS
 *
 * Sets the GPS EN pin to LOW to enable the GPS
 */
void enableGPS()
{
    SerialGPS.begin(9600);
    digitalWrite(GPS_EN, LOW); //turn on GPS so it can start satallite aquisition
}

void gpsInterrupt()
{
    if(ppsInt) ppsInt = false;
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
 * @todo Failing to get pulse sometimes
 * 
 * @returns Whether the operation was successful
 */
bool getGPS(unsigned long waittime)
{
    if(!GPS_ENABLE)
    {
        Serial.println("GPS disabled");
        return false;
    }
    unsigned long starttime = millis();
    enableGPS();
    Serial.println("Trying to get gps...");
    while (elapsedTime(starttime) < (waittime * 1000))
    {
        GPS.read();
        //    Serial.print(c);
        if (GPS.newNMEAreceived())
        {
            char * nmea = GPS.lastNMEA(); // this also sets the newNMEAreceived() flag to false
            //Serial.println(nmea);
            if (GPS.parse(nmea))
            {
                if (GPS.fix) //if we have a GPS fix...
                {
                    Serial.println("got a fix!");
                    Serial.print("searching for signal...");
                    
                    ppsInt = true;
                    attachInterrupt(digitalPinToInterrupt(PPS_PIN), gpsInterrupt, FALLING);
                    
                    while(ppsInt && elapsedTime(starttime) < (waittime * 1000));
                    if(ppsInt)
                    {
                        error("Failed to get pulse", &Serial);
                    }
                    
                    //detachInterrupt is apparently incredibly broken. who knew?
                    
                    rtc.setTime(GPS.hour, GPS.minute, GPS.seconds);
                    rtc.setDate(GPS.day, GPS.month, GPS.year);

                    Serial.print("Time: ");
                    Serial.println(generateTimestamp());


                    //set all variables to new values!
                    latitude = GPS.latitude;
                    ns = GPS.lat;
                    longitude = GPS.longitude;
                    ew = GPS.lon;
                    hdop = GPS.HDOP;

                    gpsShutdown();
                    return true;
                }
            }
        }
    }
    error("Failed to get GPS", &Serial);
    gpsShutdown();
    return false; //we didn't get a fix in the allotted time.
}

/**
 * @brief Shutdown the GPS
 *
 * The GPS is shutdown. This is done to conserve power.
 */
void gpsShutdown()
{
    GPS.fix = false;
    digitalWrite(GPS_EN, HIGH); //turn off GPS to conserve power
    //Alternative?? _rx_buffer_head = _rx_buffer_tail;
    SerialGPS.end();
    //Set lineidx in Adafruit_GPS to 0
    extern uint8_t lineidx;
    lineidx = 0;
}

/**
 * @brief Gets the current acceleration
 *
 * Accesses the accelerometer and returns the current acceleration
 *
 * @return the current acceleration
 */
double getAccel()
{
    sensors_event_t event; //sets up an event structure within the Accelerometer library
    mma.getEvent(&event); //reads the current accleration
    double ax = event.acceleration.x; //define x accel
    double ay = event.acceleration.y; //define y accel
    double az = event.acceleration.z; //define z accel
    double mag = abs(sqrt((ax * ax) + (ay * ay) + (az * az)) - 9.8); //calculate magnitude of acceleration and subtract gravity;
    return mag; //return the value of magnitude of acceleration
}

/**
 * @brief Returns the battery voltage
 *
 * 4.2V is full charge, 3.0V is dead
 */
double getBatt()
{
    double measuredvbat = analogRead(A7); //read A7, the pin that spits back voltage
    measuredvbat *= 2;    // we divided by 2, so multiply back
    measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
    measuredvbat /= 1024; // convert to voltage
    return measuredvbat; //return voltage
}

/**
 * @brief Chooses the action to take based on time
 *
 * Different times call for different actions.
 *
 * @return integer that corresponds to appropriate action
 *
 * @sa loop()
 */
int isImportantTime()
{
    uint8_t mins = rtc.getMinutes();
    if (mins % GPS_TIME == 0)  //if minutes/15 has no remainder (i.e., every 15 minutes), and its the 30th second in the minute, then its time to get GPS data.
    {
        //its time to get GPS data AND ping
        return 2;
    }
    else if (mins % PING_TIME == 0) //if minutes/5 has no remainder (i.e., every 5 minutes), and its the first second in the minute, then its time to ping other lemurs.
    {
        //its time to ping other lemurs
        return 1;
    }
    if (mins == 59) //listen for master
    {
        //Listen for master for 1 minute
        return 3;
    }
    else
    {
        return 0; //not important time
    }
}

/**
 * @brief Generates a timestamp based on the date and time.
 * 
 * @return The timestamp
 */
String generateTimestamp()
{
    String out;
    if (DMY)
    {
        out = String(rtc.getDay(), DEC) + "/" + String(rtc.getMonth(), DEC);
    }
    else
    {
        out = String(rtc.getMonth(), DEC) + "/" + String(rtc.getDay(), DEC);
    }
    out += "/" + String(rtc.getYear(), DEC) + ", " + String(rtc.getHours(), DEC) + ":" + String(rtc.getMinutes(), DEC) + ":" + String(rtc.getSeconds(), DEC);
    return out;
}

/**
 * @brief Returns formatted string of tracking data
 *
 * Takefs tracking data and creates a comma separated String to be written to the log file.
 * Logs acceleration, time, nearby nodes, location, and battery voltage.
 *
 * @param[in] accel acceleration
 * @param[in] nearby Nearby node numbers
 * @param[in] power Battery voltage
 *
 * @return Formatted data string
 */
String dataformatter(const double accel, const uint8_t * const nearbyNodes, const uint8_t nodeCount, const double power,const int * const rssi_list)
{
    String longout = "";
    String latout = "";
    
    //using the format: Date,Time,{GPS Data},HDOP,Acceleration,Battery Voltage,Nearby Nodes
    String StringtoWrite = generateTimestamp();
    StringtoWrite += "," + formatGPS(outputType, seperateDirections, latitude, ns, longitude, ew) + "," + String(hdop, 2) + "," + String(accel, 2) + "," + power + ", ";
    for(uint8_t i = 0; i < nodeCount; i++)
    {
        StringtoWrite += " " + String(nearbyNodes[i], DEC);
    }
    for(uint8_t i = 0; i < nodeCount; i++)
    {
        StringtoWrite += " " + String(rssi_list[i], DEC);
    }
    return StringtoWrite;
}

/**
 * @brief Writes a line of data to the log file
 */
void writeToSD(String input)
{
    bool wasOn = isSDOn();
    if(!wasOn)turnSDOn();
    SdFile file;
    if(!file.open("data.csv", O_WRITE | O_CREAT | O_APPEND))//open the file "data.csv". If it doesn't exist, make it.
    {
        Serial.print("Failed to open file");
        return;
    }
     file.println(input); //print data to file
     file.close(); //close the file
    if(!wasOn)turnSDOff();
}

/**
 * @brief Wipes all data except nodenumber
 *
 * Deletes all files on SD except the nodenumber
 */
void wipeData()
{
    SdFile root;
    if(root.open("data.csv"))
    {
      root.remove();
      root.close();
    }
    else
    {
      Serial.println("Failed to delete data.csv");
    }
}

//inturrupt vector for the pinging system
void ping_IRQ()
{
    continuePing = false;
}

/**
 * @brief Pings nearby collars
 *
 * Broadcast message to all nearby collars in an attempt to find its neighbors.
 * If the neighbors are close enough, they are added to the string of nodes.
 *
 * @return String of all nearby nodes
 */
bool ping(uint8_t * const nodeList, uint8_t * const nodeCount,int * const rssi_list)
{
    if(!RADIO_ENABLE) return false;
    uint8_t lastwhois = 0;
    *nodeCount = 0;
    setRadioLowPower(); //reduce power on radio
    delay(50); //wait for others to catch up.

    rtc.disableAlarm();
    rtc.detachInterrupt();

    uint8_t sec = rtc.getSeconds();
    
    //Attempt to sync the start of ping
    //Note: The first second, noone will broadcast. This is intentional.
    rtc.setAlarmTime(0, 0, (sec + 1) % 60);
    rtc.enableAlarm(rtc.MATCH_SS);
    rtc.attachInterrupt(ping_IRQ); //attach Inturrupt vector

    continuePing = true;
    while(continuePing);
    
    rtc.disableAlarm();
    rtc.detachInterrupt();

    if(nodenumber != 1)
    {
      /*  rtc.disableAlarm();
        rtc.detachInterrupt();

        //Wait nodenumber number of seconds before broadcasting. Until the receive.
        rtc.setAlarmTime(0, 0, sec + nodenumber);
        rtc.enableAlarm(rtc.MATCH_SS);
        rtc.attachInterrupt(ping_IRQ); //attach Inturrupt vector*/

        unsigned long startTime = millis();
        while(elapsedTime(startTime) < 1000*(nodenumber - 1))
        {
            if(manager.available())
            {
                uint8_t msg;
                uint8_t size = sizeof(msg);
                if(manager.recvfrom(&msg, &size) && msg != lastwhois && *nodeCount <= 254)
                {
                    int16_t rssi = rf69.lastRssi();
                    if(rssi >= RSSI_THRESHOLD)
                    {
                       
                        lastwhois = msg;
                        nodeList[*nodeCount] = msg;
                        rssi_list[*nodeCount] = rssi;
                        (*nodeCount)++;
                    }
                }
            }
        }
    }

 /*   rtc.disableAlarm();
    rtc.detachInterrupt();

    //Broadcast for <=1 second
    rtc.setAlarmTime(0, 0, sec + nodenumber + 1);
    rtc.enableAlarm(rtc.MATCH_SS);
    rtc.attachInterrupt(ping_IRQ); //attach Inturrupt vector*/

    unsigned long startTime = millis();
    while(elapsedTime(startTime) < 1000)
    {
        manager.sendto(&nodenumber, sizeof(nodenumber), RH_BROADCAST_ADDRESS);
        if(!manager.waitPacketSent(1000))
        {
            Serial.println("Timeout");
        }
    }

    if(nodenumber != NUMBER_OF_NODES)
    {
    /*    rtc.disableAlarm();
        rtc.detachInterrupt();

        //Listen for rest of pings
        rtc.setAlarmTime(0, 0, sec + NUMBER_OF_NODES + 1);
        rtc.enableAlarm(rtc.MATCH_SS);
        rtc.attachInterrupt(ping_IRQ); //attach Inturrupt vector*/

        startTime = millis();
        while(elapsedTime(startTime) < 1000*(NUMBER_OF_NODES - nodenumber))
        {
            if(manager.available())
            {
                uint8_t msg;
                uint8_t size = sizeof(msg);
                if(manager.recvfrom(&msg, &size) && msg != lastwhois && *nodeCount <= 254)
                {
                    int16_t rssi = rf69.lastRssi();
                    if(rssi >= RSSI_THRESHOLD)
                    {
                        lastwhois = msg;
                        nodeList[*nodeCount] = msg;
                        rssi_list[*nodeCount] = rssi;

                        (*nodeCount)++;
                    }
                }
            }
        }
    }

    default_timer();
    setRadioHighPower(); //increase power on radio
    Serial.println("Done Pinging.");
    return true; //return this
}

/**
 * @brief Enables Emergency Low Power Mode
 *
 * On the event that the battery's voltage drops too low, shutsdown the Arduino.
 * Wakes every hour to check voltage.
 */
void outofPower()
{
    //Whoops! We've run out of power!

    //Write to SD that we ran out of power, then only wake up once every hour to check power
   // turnSDOn();
    error("Out of Power Emergency Shutdown!", &Serial);
    turnSDOff();
    
    turnRadioOff();
    
    //disable old inturrupt
    rtc.disableAlarm(); //Wake when the seconds match
    rtc.detachInterrupt(); //attach Inturrupt vector

    //set new inturrupt
    rtc.setAlarmTime(0, 59, 59); //calls the update on the 59th second and 59th minute so every hour at the top of the hour
    rtc.enableAlarm(rtc.MATCH_MMSS); //Wake once per hour
    rtc.attachInterrupt(oopirq); //attach Inturrupt vector

    bool poweredup = false; //set variable for when we have enough power
    while (!poweredup) //while we are still under our threshhold
    {
        double batt = getBatt();
        if (batt > 3.3) //if we are above "normal voltage" threshold
        {
            poweredup = true; //we're good on power
        }
        else //if we are still too low on power
        {
            rtc.standbyMode(); //sleep until next hour
        }
    }

    // by this point, we should be good on power. Now lets:

    //disable new inturrupt
    rtc.disableAlarm(); //Wake once per hour
    rtc.detachInterrupt(); //attach Inturrupt vector

    //reenable old inturrupt
    rtc.setAlarmTime(0, 0, 59); //calls the update on the 59th second so the time is 00.
    rtc.enableAlarm(rtc.MATCH_SS); //Wake when the seconds match
    rtc.attachInterrupt(irq); //attach Inturrupt vector
}

//this is the inturrupt vector for the out of power system. Does not do anything, but keeps timing by its very existance.
void oopirq() 
{
	//nothing here
}

//this function shuts the node down until a distanct future. this will shut down for a long, long time, only turning on once per day to get GPS data and update its RTC.
/**
 * @brief Shutsdown collar for a long time
 *
 * Shuts down collar until a given date.
 * Wakes every day at noon to check for instructions from base.
 */
void extendedShutdown()
{
    turnRadioOff();
    turnSDOff();
    rtc.disableAlarm(); //shuts the old alarm off
    rtc.detachInterrupt(); //detatch old inturrupt vector
    rtc.setAlarmTime(11, 59, 0); //calls the update one minute before noon
    rtc.enableAlarm(rtc.MATCH_HHMMSS); //sets new alarm to wake up once per day
    rtc.attachInterrupt(extendedAlarm); //set new inturrupt vector
    bool escape = false; //we are entering the extended shutdown loop.
    while (!escape)
    {
        rtc.standbyMode(); //see you in the future, kiddo
        getGPS(59); //update GPS and RTC
        //check if its time to wake up.
        if ((rtc.getMonth() == mn) && (rtc.getDay() == da) && (rtc.getYear() == yr))
        {
            //wake up!
            escape = true; //escape from extended shutdown loop
        }
        listenForMaster(59000, &escape);//listen on radio for 60 seconds;
    }
    rtc.disableAlarm(); //shuts the new alarm off
    rtc.detachInterrupt(); //detatch new inturrupt vector
    rtc.setAlarmTime(0, 0, 59); //calls the update on the 59th second so the time is 00.
    rtc.enableAlarm(rtc.MATCH_SS); //Wake when the seconds match
    rtc.attachInterrupt(irq); //attach Inturrupt vector
    rtc.standbyMode(); //sleep
}

//an inturrupt called by the extended shutdown. Does not do anything, but keeps timing by its very existance.
void extendedAlarm() 
{
	//nothing here
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
 * @brief Listens for base station
 *
 * Listens for base station for specified wait time.
 * Also listens for a "wake" call to end an extended shutdown.
 *
 * @param[in] timetowait Time to listen for master before giving up.
 * @param[out] def_wake Whether a wake command was received. Defaults to nullptr
 */
void listenForMaster(int timetowait, bool * def_wake)
{
    setRadioHighPower();
    Serial.print("Listening for master for ");
    Serial.print(timetowait/1000);
    Serial.println(" seconds");
    uint8_t to = RH_BROADCAST_ADDRESS;
    String input = receive(timetowait, &to);
    Serial.println("input:");
    Serial.println(input);
    if (input == "start")
    {
        Serial.println("I received the start command.");
        String outnum = String(nodenumber, DEC);
        delay(500*(nodenumber - 1)); //delay a little bit to let the master clock them all in
        Serial.println("Our turn to transmit");
        transmit(outnum, 0);
        //wake from extended shutdown
        if(def_wake != nullptr)
        {
            *def_wake = true; //we can escape the extended shutdown loop.
        }
        handshake(); //go into handshake mode to talk to master!
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
    transmit(in, RH_BROADCAST_ADDRESS);
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
    Serial.println(in);
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
 * @param[out] to The collar the message was addressed to (this or broadcast)
 *
 * @return The message or an empty String if no message is received.
 */
String receive(uint16_t timeout, uint8_t * to)
{
    uint8_t len = sizeof(buf);
   // Serial.println("Receiving...");
    if(manager.recvfromAckTimeout(buf, &len, timeout, nullptr, to))
    {
        Serial.println("Received message");
        return (char*)buf;
    }
  //  Serial.println("Message timed out");
    return "";
}

/**
 * @brief Sets the radio to low power
 *
 * Radio power ranges from 14 to 20, this sets it to 14.
 * Used to talk to nearby collars.
 */
void setRadioLowPower()
{
    rf69.setTxPower(14, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW
}

/**
 * @brief Sets the radio to high power
 *
 * Radio power ranges from 14 to 20, this sets it to 20.
 * Used to talk to the base station.
 */
void setRadioHighPower()
{
    rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW
}

/**
 * @brief Returns the acceleration
 *
 * Returns either average or max acceleration over a second depending on global config value.
 */
double avAccel()
{
    double acceleration = 0;
    unsigned long elapsed = 0;
    unsigned long starttime = millis();
    if (ACCEL_TYPE) //if we want the average over a second...
    {
        int count = 0;
        double sumAcc = 0;
        while (elapsed < 1000)
        {
            sumAcc = sumAcc + getAccel();
            count++;
            elapsed = elapsedTime(starttime);
        }
        acceleration = sumAcc / count;
    }
    else  //if we want the max over 1 second...
    {
        double maxAcc = 0;
        while (elapsed < 1000)
        {
            double newAcc = getAccel();
            if (newAcc > maxAcc)
            {
                maxAcc = newAcc;
            }
            elapsed = elapsedTime(starttime);
        }
        acceleration = maxAcc;
    }
    return acceleration;
}

/**
 * @brief Returns amount of free RAM
 *
 * @note
 *  This is a hack. May not be reliable.
 */
uint32_t freeRam()
{
    char stack_dummy = 0;
    return &stack_dummy - sbrk(0);
}

/**
 * @brief Uploads tracking data to base station
 *
 * Loads the tracking data into RAM and sends it back to the base station over radio.
 */
void uploadData()
{
  //  turnSDOn();
    Serial.println("uploading...");
    setRadioHighPower();
    SdFile file;
    if(!file.open("data.csv"))
    {
        Serial.println("Failed to open file");
        manager.setHeaderFlags(1);
        broadcast("Finished!"); //say that we are done.
        //The first bit is cleared
        manager.setHeaderFlags(0, 1);
        return;
    }
    Serial.println("File opened!");
    
    uint32_t file_size = file.fileSize();
    //Max acceptable buffer size
    uint32_t buf_size = (freeRam() / 10) * 4;
    if (file_size < buf_size)
    {
        buf_size = file_size;
    }
    
    Serial.print("Buffer size: ");
    Serial.println(buf_size);
    
    uint8_t* const buf = new uint8_t[buf_size];
    
    int number_of_bytes;
    while (number_of_bytes = file.read(buf, buf_size), number_of_bytes > 0)
    {
        uint8_t * buf_small = buf;
        while (buf_small < buf + number_of_bytes)
        {
            int packet_size = RH_RF69_MAX_MESSAGE_LEN;
            int remaining = buf + number_of_bytes - buf_small;
            if (remaining < packet_size)
            {
                packet_size = remaining;
            }
            if(!manager.sendtoWait(buf_small, packet_size, 0))
            {
                error("Lost connection to base station", &Serial);
                //Breaks out a goto
                //You can't stop me
                goto stop_transfer;
            }
            buf_small += RH_RF69_MAX_MESSAGE_LEN;
        }
    }
stop_transfer:
    delete [] buf;
    file.close(); //close the file
  //  turnSDOff();
    //Sets the header flags with a bitmask of 1.
    //The first bit marks the packet as an end message
    manager.setHeaderFlags(1);
    broadcast("Finished!"); //say that we are done.
    //The first bit is cleared
    manager.setHeaderFlags(0, 1);
}

/**
 * @brief Resets the radio
 *
 * Toggles the radio reset pin using the recommended? delay of 10ms
 *
 * @todo Low Priority, check 10ms claim
 */
void restartradio()
{
    digitalWrite(RFM69_RST, HIGH);
    delay(10);
    digitalWrite(RFM69_RST, LOW);
    delay(10);
}

/**
 * @brief Sets radio to idle.
 *
 * @note
 * Not needed currently
 */
void turnRadioOn()
{
    Serial.println("Turning radio on");
    digitalWrite(RFM69_CS, HIGH);
}

/**
 * @brief Puts radio in deep sleep
 *
 * @note
 * Slight delay on coming up, nothing to worry about
 */
void turnRadioOff()
{
    Serial.println("Turning radio off");
    digitalWrite(RFM69_CS, LOW);
}

/**
 * @brief Enables SD card reader
 */
void turnSDOn()
{
    Serial.println("Turning SD on");
    digitalWrite(SD_CS, HIGH);
    delay(150);
}

/**
 * @brief Disables SD card reader
 */
void turnSDOff()
{
    Serial.println("Turning SD off");
    digitalWrite(SD_CS, LOW);
}

/**
 * @brief Returns true if the SD card is on
 */
bool isSDOn()
{
    return digitalRead(SD_CS) == HIGH ? true : false;
}

/*
 * @brief blinks the LED for X seconds.
 * 
 * blinks an LED attached to pin 13. This is called by the blink function so a single node can be determined. Also used to alert operator of conditions worthy of intervention. 
 *
 * @param[in] Flash time. How long does the LED flash?
 */
void flash(unsigned int flashtime)
{
	const unsigned int flashrate = 2; 	//flash rate in flashes per second. If you really care you can change this. Set by default to 2 flashes per second.
	unsigned long startTime = millis(); //set the start time of our timer
	while (elapsedTime(startTime) < (flashtime*1000)) //while we are less than our flash time designation
	{
		digitalWrite(LED, HIGH);//turn LED on
		delay(500/flashrate); //wait
		digitalWrite(LED, LOW);//turn LED off
		delay(500/flashrate); //wait
	}
}

/**
 * @brief Returns temperature of the battery in celcius from a 100k Thermistor
 * 
 * Used with an NTC Epcos 100k Thermistor with a 100k ohm resistor on the other end of the voltage divider. If you are using another themistor, edit the equations below with the correct Steinhart-Hart equation. If the battery temperature raises beyond a safe level determined by the "safeTemp" variable, the function toggles a bool, Panic, to indicate the node is having battery troubles.
 * 
 * @return Temperature in Celcius
 */
double checkBattTemp()
{
  if (useThermistor)
  {
    int rawADC = analogRead(THERM); //read in value from the 10-bit ADC
    double resistance = log(100000.0 * ((1024.0 / rawADC - 1))); //turn ADC value into the log of resistiance
    double Temp = 1/(0.001*0.5379336721+(0.0001*2.209214900+(1.842764786*0.0000001*resistance*resistance))*resistance);    //reformatted thermistor equation called the Steinhart-Hart equation.  
    Temp = Temp - 273.15; //convert from Kelvin to Celcius 
    return Temp;
  }
  else
  {
    return 0; //not using the thermistor
  }
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
