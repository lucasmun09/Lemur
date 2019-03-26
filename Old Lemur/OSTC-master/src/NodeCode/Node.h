
/** @file */

#ifndef OSTC_NODE_H
#define OSTC_NODE_H

#include <OSTC_GPS.h>

//Global Variables used for adapting behavior

//Pins
/**Radio Slave Select Pin*/
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

//Radio
/**Frequency to set Radio to
 * 
 * The RFM69 has 4 frequency ranges:
 *  - 290-340 MHz   (315 MHz)
 *  - 424-510 Mhz   (433 Mhz)
 *  - 862-890 Mhz   (868 Mhz)
 *  - 890-1020 Mhz  (915 Mhz)
 * 
 * @note 915 is an ISM band in Region 2 (America),
 * and 433 is an ISM band Region 1 (Europe, Africa)
 */
static const float RF69_FREQ = 915.0;

//Debugging variables
/**Enable the GPS module*/
static const bool GPS_ENABLE = true;
/**Enable the Radio*/
static const bool RADIO_ENABLE = true;
/**Enable the Serial for debugging*/
static const bool SERIAL_ENABLE = true;
/**When powered, wait for USB serial to connect before starting setup
 * This allows for easier debugging
 */
static const bool WAIT_FOR_DEBUG = false;
/**Skip RTC sleep in loop for debugging*/
static const bool NO_WAIT = true;
/**Set the loop action for debugging
 *  - 0 for normal behavior
 *  - 1 ping
 *  - 2 ping + GPS
 *  - 3 listen for base
 */
static const short DEBUG_IMPORTANCE = 0;

/**below this voltage, all data recording will stop until the power reaches 3.7V*/
static const double POWER_THRESH = 3.3;

/**Battery Safety Variables**/
//Are we using the thermistor? Not really required for use, but its a good safeguard to alert the operator of failing batteries when dealing with endangered animals. Could in theory be used to get temperature data. See the manual for how to assemble the thermistor circuit.
static bool useThermistor = false; 
// the highest temperature the battery can get (in Celcius) before we can assume it is failing and the node should try and contact the master.
static int safeTemp = 45; 

/**Display format as month/day/year (True) or day/month/year (False)?*/
static const bool DMY = false;

/**If a Flash command is called, how long should the LED blink for? The Base station will always show 10 seconds, but this changes that number*/
static const int flashDuration = 10;

/**Interval in minutes to ping (search for nearby collars)*/
static const int PING_TIME = 5;
/**Interval in minutes to update GPS*/
static const int GPS_TIME = 15;
/**Interval in minutes to write to SD*/
static const int SD_CARD_WRITE_TIME = 10;

/**The total number of Nodes. The max is 254*/
static const uint8_t NUMBER_OF_NODES = 3;

/**The RSSI threshold defined as "close"
 * Used to determine whether collars are nearby
 * 
 * -27 is really close, and -91 is when we lose signal. The higher this value (the closer to positive), the closer the nodes have to be to be logged as "nearby".
 */
static const int RSSI_THRESHOLD = -35;

/**Set the type of acceleration to measure
 *  - 0 Max acceleration over 1 second
 *  - 1 Average acceleration over 1 second
 */
static const bool ACCEL_TYPE = 1;


/**Set the range of accelerations to measure in Gs. 4g is good for most applications, but for slow animals, 2g gives a more accurate value. For fast animals like birds, 8g will give more range of values.
 *  - 0 +/- 2g
 *  - 1 +/- 4g
 *  - 2 +/- 8g
 */
 static const int accelRange = 1;

/* Set the output data type: Latitude/Longitude (in DD), Latitude/Longitude (in DMS), or UTM WGS84. 
 * 
 *  - DecDeg  Latitude/Longitude in Decimal Degrees (e.g.  37.228718N,80.423132W)
 *  - DMS     Latitude/Longitude in Degrees-Minutes-Seconds (e.g.  37°13'43.55"N,80°25'23.27"W)
 *  - UTM     Universal Transverse Mercator (e.g.  17S,551173.4,4120401.1)
 */
static const LocationFormat outputType = DecDeg;

/* Do you want to seperate direction letters (N,S,E,W) from the actual number? 
 * This will put them in a seperate column when the CSV is opened in Excel. 
 * Doesn't change information in any way, but affects how it is formatted. 
 * (For UTM, this seperates the zone letter from the zone number)
 * 
 * - True   Letters WILL be seperated (e.g.  37.228718,N,80.423132,W)
 * - False  Letters WILL NOT be seperated (e.g.  37.228718N,80.423132W)
 */
static const bool seperateDirections = false;


//Encryption Key. It has to be 16 bytes and the same as the base and other nodes.
//uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
#endif
