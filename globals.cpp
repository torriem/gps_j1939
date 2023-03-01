#include "globals.h"
#include "conversions.h"

//define and initialize all the variables externed in globals.h

/********************************
 * Antenna physical configuration
 ********************************/

double antenna_forward = 0;
double antenna_height = 120 * INCHES;
double antenna_right = 26.5 * INCHES;

/*****************************
 * IMU configuration and state
 *****************************/

int imu_source = 0;
float imu_roll_offset = 0;
bool imu_use_pitch = false;
bool imu_reverse = false;
bool imu_heading_offset_set = false;
float imu_heading_offset = 400;
int imu_lookback = 90;
float imu_last_roll = 0;

/*****************************
 * GPS configuration and state 
 *****************************/

double gps_latitude;
double gps_longitude;
double gps_orig_latitude;
double gps_orig_longitude;
double gps_heading;
double gps_speed;
double gps_altitude;
double gps_orig_altitude;
uint64_t gps_j1939_datetime;
float gps_yawrate; //comes from IMU but through a GPS implementation
float gps_roll; //comes from IMU, but through a GPS implementation
int gps_mode; //GPS mode as defined in GGA

char gps_fix_time[12]; //HHMMSS.ss
char gps_fix_date[12]; //DDMMYY
float gps_dgps_age;
int gps_num_sats;
float gps_hdop;
float gps_geoid;

/*
 * Serial NMEA outputs
 */

#if defined(ESP32)
//These must be configured in the main program with the appropriate
//Serial device instances.
SerialNMEA bluetooth_nmea = SerialNMEA(0,"Bluetooth",NULL);
SerialNMEA serial1_nmea = SerialNMEA(1,"Serial1", NULL);
//SerialNMEA serial2_nmea = SerialNMEA(2,"Serial2", NULL);
//SerialNMEA serial3_nmea = SerialNMEA(3,"Serial3", NULL);
#elif defined(TEENSY)
SerialNMEA usbserial_nmea;
#endif


//generate corrected sentences or pass through from GPS
bool gps_generate_gga;
bool gps_generate_vtg;
bool gps_generate_panda;



