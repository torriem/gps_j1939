#include "globals.h"
#include "defaults.h"

//define and initialize all the variables externed in globals.h

/********************************
 * Antenna physical configuration
 ********************************/

double antenna_forward = ANTENNA_DEFAULT_FORWARD;
double antenna_height = ANTENNA_DEFAULT_HEIGHT;
double antenna_right = ANTENNA_DEFAULT_RIGHT;

/*****************************
 * IMU configuration and state
 *****************************/

int imu_source = IMU_DEFAULT_SOURCE;
float imu_roll_offset = IMU_DEFAULT_ROLL_OFFSET;
bool imu_use_pitch = false;
bool imu_reverse = false;
bool imu_heading_offset_set = false;
float imu_heading_offset = 400;
int imu_lookback = IMU_DEFAULT_LOOKBACK;
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


