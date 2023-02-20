#ifndef __NMEAIMU_H__
#define __NMEAIMU_H__
#include "Arduino.h"
#include "fix_handler.h"

void setup_nmea_parser(FixHandler fix_handler, Stream *imu_stream);
void nmea_process(char c);
int16_t get_imu_lookback(void);
void set_imu_lookback(int16_t);

#endif


