#ifndef __NMEAIMU_H__
#define __NMEAIMU_H__
#include "Arduino.h"
#include "fix_handler.h"

#define ROLL_NORMAL 1
#define ROLL_REVERSE -1

//extern bool swap_pitch_roll;
//extern float roll_offset;
//extern float roll_direction;

//TODO: separate out the IMU reader into its own
//      module so we can work the WT901c later

void setup_nmea_parser(FixHandler fix_handler, Stream *imu_stream, SendNMEA send_nmea_handler);
void nmea_process(char c);
void read_imu();
int16_t get_imu_lookback(void);
void set_imu_lookback(int16_t);

#endif


