#ifndef __DEFAULTS_H__
#define __DEFAULTS_H__

#include "whichteensy.h"
#include "conversions.h"

/* Here are the default values used in other parts of the sketch.
 * Hopefully eventually many will be configurable at run time, and
 * stored in the EEPROM/Flash.
 *
 * But here are the starting values.  Unfortunately the possible
 * values are scattered throughout the other files at this time.
 * TODO: clean them up and put them here.
 */

/* Definitions */
//types of GPS
#define GPS_PX1172RH 1 //dual gps from px1172rh
//#define GPS_DUAL_F9P //FUTURE: dual F9P support
#define GPS_NMEA_BNO 3  //single GPS GGA+VTG and the BNO08x

//types of virtual position generators
#define VIRTUAL_NONE 0 //no virtual position generation
#define VIRTUAL_CIRCLE 10 //simulate moving in a perfect circle
#define VIRTUAL_STATIC 11 //arbitrary static position.

//IMU sources (TODO: not implemented yet
#define IMU_BNO 0

//Serial IDs
#if defined(TEENSY)
#define SERIAL_ID_USB 0
#define SERIAL_ID_OUT 1

#elif defined(ESP32)
#define SERIAL_ID_BT 0
#define SERIAL_ID_OUT 1
#endif

/* Default values */
//ANTENNA placement
#define ANTENNA_DEFAULT_FORWARD 0
#define ANTENNA_DEFAULT_HEIGHT 120 * INCHES
#define ANTENNA_DEFAULT_RIGHT 26.5 * INCHES

//GPS
#define GPS_DEFAULT_SOURCE GPS_NMEA_BNO // GPS_PX1172RH
#define GPS_TIMEOUT 1000 //how long to wait until sending "No GPS messages."
#define GPS_DEFAULT_VIRTUAL_SOURCE VIRTUAL_NONE

//IMU
#define IMU_DEFAULT_SOURCE IMU_BNO
#define IMU_DEFAULT_ROLL_OFFSET -3.625
#define IMU_DEFAULT_LOOKBACK 90
#define IMU_DEFAULT_WINDOW 3

//Serial ports
#if defined(TEENSY)
#define SERIAL_DEFAULT_IMU Serial //for testing, otherwise use Serial4
#define SERIAL_DEFAULT_GPS Serial3
#define SERIAL_DEFAULT_OUT Serial5

#define SERIAL_DEFAULT_OUT_SPEED 115200
#define SERIAL_DEFAULT_GPS_SPEED 460800 //on the px1172rh here
#define SERIAL_DEFAULT_IMU_SPEED 115200


#elif defined(ESP32)
#define SERIAL_DEFAULT_OUT 0
#define SERIAL_DEFAULT_IMU 1
#define SERIAL_DEFAULT_GPS 2

#define SERIAL_DEFAULT_OUT_SPEED 115200
#define SERIAL_DEFAULT_GPS_SPEED 460800 //f9p 
#define SERIAL_DEFAULT_IMU_SPEED 115200

#define BLUETOOTH_DEFAULT_NAME "rovertest"

#define IMU_RX 27
#define IMU_TX 16

#define GPS_RX 25
#define GPS_TX 14

#define OUT_RX 3
#define OUT_TX 1
#endif

#endif
