#ifndef __NMEAIMU_H__
#define __NMEAIMU_H__
#include "Arduino.h"
#include "fix_handler.h"
#include "imubase.h"

namespace nmea_imu {
	void setup(FixHandler fix_handler = NULL, IMUBase *the_imu = NULL);
	void set_imu(IMUBase *the_imu);
	void set_on_fix_handler(FixHandler fix_handler);
	void process_byte(char c);
}
#endif


