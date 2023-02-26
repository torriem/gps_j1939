#ifndef __CIRCLE_GENERATOR_H__
#define __CIRCLE_GENERATOR_H__

#include <Arduino.h>

/* This class is a latitude and longitude generator that
 * virtually drives in a big circle to allow us to set
 * a circle track on the brown box monitor according.
 * This is intended for our pivot circles where the outer-
 * most boundary is a known radius from the pivot point.
 */

class CircleGenerator {
protected:
	double center_lat;
	double center_lon;
	double center_alt;

	double radius;
	double speed;
	double yaw_rate; //calculate from speed and radius
	double delta_angle;
	double current_angle;
	unsigned long start_time;
	unsigned long timeout;
	unsigned long last_message_time;

public:
	CircleGenerator() { }

	inline void set_circle(double lat, double lon, double drive_radius, double alt) {
		center_lat = lat;
		center_lon = lon;
		center_alt = alt;
		radius = drive_radius;
	}
			
	inline void set_radius(double radius_meters) {
		radius = radius_meters;
	}

	inline bool finished(unsigned long time) {
		return ((time - start_time) > timeout);
	}

	//reset the generator, calculate everything we need to run
	void start(unsigned long begin_time, uint32_t timeout, double speed);

	//calculate a new position if enough time has elapsed
	//put it in the autosteer_* global variables
	bool circle_position(unsigned long time, uint32_t cycle_time);
};

#endif
