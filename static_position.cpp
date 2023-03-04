#include "static_position.h"
#include "globals.h"

bool StaticPosition::static_position(unsigned long time, uint32_t cycle_time) {
	if ((time - start_time) >= timeout)
		return false; //no more generated positions

	if ((time - last_message_time) >= cycle_time) {
		last_message_time = time;

		gps_mode = 4;
		gps_latitude = latitude;
		gps_longitude = longitude;
		gps_heading = heading;
		gps_roll = 0;
		gps_speed = 0;
		gps_altitude = altitude;

		//fake time from teensy millis() timer
		uint32_t seconds = (time % 60000);
		uint32_t minutes = ((time - seconds) / 60000) % 60;
		uint32_t hours = (time - seconds) / 3600000;

		//jan 1, 1985
		gps_j1939_datetime = (uint64_t)(((float)time / 1000.0) * 4);
		gps_j1939_datetime |= ((uint64_t)minutes << 8);
		gps_j1939_datetime |= ((uint64_t)hours << 16);
		gps_j1939_datetime |= 0x7d7d000401000000;

		return true;
	}
	return false;
}

