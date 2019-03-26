#ifndef OSTC_GPS
#define OSTC_GPS

#include <Arduino.h>

enum _LocationFormat {
    DecDeg,
    DMS,
    UTM
};
typedef enum _LocationFormat LocationFormat;

/**
 * @brief Formats GPS data to desired format
 *
 * Outputs a string that is formatted for data storage
 *
 * @param[in] Format Desired output format
 * @param[in] SeperateDirections
 *              Seperate the direction (N S E W) into its own column.
 *              For UTM, seperate zone letter from number
 * @param[in] Latitude in NMEA format DDDMM.MMM
 * @param[in] NorS North or South
 * @param[in] Longitude in NMEA format DDDMM.MMM
 * @param[in] EorW East or West
 *
 * @return the output string.
 */
String formatGPS(LocationFormat Format, bool SeperateDirections, float Latitude, char NorS, float Longitude, char EorW);
#endif
