#ifndef __NMEAIMU_H__
#define __NMEAIMU_H__
#include "Arduino.h"
#include "fix_handler.h"

void setup_nmea_parser(FixHandler fix_handler, Stream *imu_stream);
void nmea_process(char c);

#endif


