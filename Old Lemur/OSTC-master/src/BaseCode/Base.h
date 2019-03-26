/** @file */

#ifndef OSTC_BASE_H
#define OSTC_BASE_H

#include <OSTC_GPS.h>

//Debug flags
/**When powered, wait for USB serial to connect before starting setup
 * This allows for easier debugging
 */
static const bool WAIT_FOR_DEBUG = false;

//Pins
/**Radio Slave Select Pin*/
static const uint8_t RFM69_CS = 6;
/**Radio Interrupt Pin*/
static const uint8_t RFM69_INT = 5;
/**Radio Reset Pin*/
static const uint8_t RFM69_RST = 9;
/**GPS Enable Pin*/
static const uint8_t GPS_EN = 13;
/**GPS Pulse per Second Pin*/
static const uint8_t PPS_PIN = 3;   //GPS pulse per second pin
/**SD Slave Select Pin*/
static const uint8_t SD_CS = 4;   //SD card Enable pin. 

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

/**The total number of collars
 * 
 * The max is 254
 */
static const uint8_t NUMBER_OF_NODES = 9;

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
#endif
