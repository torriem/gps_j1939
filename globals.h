#ifndef __GLOBALS_H__
#define __GLOBALS_H__

#include <Arduino.h>
/*
 * I know everyone hates globals, but they do serve a purpose
 *
 * Here are the main global state variables that all of the various
 * GPS and IMU implementations need to work with to pass data back
 * to the main loop for sending out serial ports or the CAN bus
 */

/********************************
 * Antenna physical configuration
 ********************************/

/* For dual antennas, this refers to the primary antenna,
 * usually the right-most antenna (starboard)
 */

//distance ahead of axle (negative if behind)
extern double antenna_forward;

//height above the axle
extern double antenna_height;

//distance from the center of the vehicle to the
//antenna where positive is right and negative
//is left.
extern double antenna_right;

/*****************************
 * IMU configuration and state
 *****************************/

//which IMU source to use?  See imu.h for list
//of supported IMU devices.
extern int imu_source;

// degrees to add to the IMU roll reading to get 
// true roll angle.  Positive tips to the right,
// negative tips to the left
extern float imu_roll_offset;

//if IMU is mounted such that the roll axis is actually the
//pitch axis, use that instead of roll
extern bool imu_use_pitch;

//reverse (negate) the raw IMU reading, perhaps if the
//IMU is mounted upside down.
extern bool imu_reverse;

//do we know the offset between the IMU heading and the real
//heading yet?  Maybe don't need with 400 on imu_heading_offset
extern bool imu_heading_offset_set;

//current offset between the imu heading and the real heading
//set to 400 if offset is unknown. Read only.
extern float imu_heading_offset;

//latency of the IMU. In other words how far back in time
//to get a reading that lines up with the GPS position. 
//in milliseconds
extern int imu_lookback;

//last IMU roll reading used to correct GPS, if known.
//400 if not known.  Read only
extern float imu_last_roll;

/*****************************
 * GPS configuration and state 
 *****************************/

extern double gps_latitude;
extern double gps_longitude;
extern double gps_orig_latitude;
extern double gps_orig_longitude;
extern double gps_heading;
extern double gps_speed;
extern double gps_altitude;
extern uint64_t gps_j1939_datetime;
extern float gps_yawrate; //comes from IMU but through a GPS implementation
extern float gps_roll; //comes from IMU, but through a GPS implementation
extern int gps_mode; //same as GGA's fix type field

extern char gps_fix_time[12]; //HHMMSS.ss 
extern char gps_fix_date[12]; //DDMMYY 
extern float gps_dgps_age;
extern int gps_num_sats;
extern float gps_hdop;
extern float gps_geoid;

//generate corrected sentences or pass through from GPS
extern bool gps_generate_gga;
extern bool gps_generate_vtg;
extern bool gps_generate_panda;



#endif

