#ifndef __STATIC_POSITION_H__
#define __STATIC_POSITION_H__

#include <Arduino.h>

extern double autosteer_lat;
extern double autosteer_lon;
extern double autosteer_heading;
extern double autosteer_roll;
extern double autosteer_yawrate;
extern double autosteer_speed;
extern double autosteer_altitude;
extern uint64_t autosteer_datetime;
extern char autosteer_mode;

/* This class generates a static position, simulating a GPS
 * receiver that is not moving.  This can be used to recall
 * a certain lat/lon position for setting an AB or A+heading
 * line on a Brown Box monitor to a specific location. Kind of
 * helps to make a poor man's preset line.
 */

class StaticPosition {
private:
	double latitude;
	double longitude;
	double altitude;
	double heading;

	double last_message_time;
	
	unsigned long start_time;
	unsigned long timeout;


public:
	inline void set_position(double lat, double lon, double alt, double heading) {
		latitude = lat;
		longitude = lon;
		altitude = altitude;
		this->heading = heading;
		last_message_time = 0;
	}

	bool static_position(unsigned long time, uint32_t cycle_time);
	inline void start(unsigned long begin_time, uint32_t timeout) {
		start_time = begin_time;
		this->timeout = timeout * 1000; //convert s to ms
	}

	inline bool finished(unsigned long time) {
		return ((time - start_time) > timeout);
	}
};

#endif
