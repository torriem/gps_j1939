#include "circle_generator.h"
#include <math.h>
#include "haversine.h"

/* reset the generator back to straight north, and set
 * timeout for generation, and travel speed
 */
void CircleGenerator::start(unsigned long begin_time, uint32_t timeout, double travel_speed) {
	speed = travel_speed;
	current_angle = 0;
	yaw_rate = 360.0 / (PI * radius * 2 / speed / 1000 * 3600);  //clockwise
	last_message_time = begin_time;

	delta_angle = -1;

	start_time = begin_time;
	this->timeout = timeout * 1000; //convert s to ms
}

/* generate the next position around the circle, every
 * cycle_time ms.  Populate global autosteer variables
 */

bool CircleGenerator::circle_position(unsigned long time, uint32_t cycle_time) {

	if (delta_angle < 0) 
		//calculate it based on the cycle time
		delta_angle = yaw_rate * cycle_time / 1000; //how far to move each time

	if ((time - last_message_time) >= cycle_time) {
		double lat, lon;
		last_message_time = time;

		current_angle += delta_angle;
		if (current_angle >= 360.0) {
			//wrap and around and continue forever.
			current_angle -= 360.0;
		}
		lon = center_lon;
		lat = center_lat;
		haversine::move_distance_bearing( lat, lon, current_angle, radius );

		autosteer_mode = 'R';
		autosteer_lat = lat;
		autosteer_lon = lon;
		autosteer_heading = current_angle + 90;
		if (autosteer_heading >= 360.0) 
			autosteer_heading -= 360.0;
		autosteer_roll = 0;
		autosteer_yawrate = yaw_rate;
		autosteer_speed = speed;
		autosteer_altitude = center_alt;

		//fake time from teensy millis() timer
		uint32_t seconds = (time % 60000);
		uint32_t minutes = ((time - seconds) / 60000) % 60;
		uint32_t hours = (time - seconds) / 3600000;

		//jan 1, 1985
		autosteer_datetime = (uint64_t)(((float)time / 1000.0) * 4);
		autosteer_datetime |= ((uint64_t)minutes << 8);
		autosteer_datetime |= ((uint64_t)hours << 16);
		autosteer_datetime |= 0x7d7d000401000000;

		return true;
	}

	return false;
}
