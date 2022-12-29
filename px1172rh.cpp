#include "px1172rh.h"
#include "stinmea.h"
#include "haversine.h"
#include "kfilter1.h"
#include "SimpleKalmanFilter.h"
	
extern double antenna_forward;
extern double antenna_height;
extern double antenna_right;

extern double autosteer_lat;
extern double autosteer_lon;
extern double autosteer_heading;
extern double autosteer_roll;
extern double autosteer_yawrate;
extern double autosteer_speed;
extern double autosteer_altitude;
extern char autosteer_mode;

extern double autosteer_orig_lat;
extern double autosteer_orig_lon;
extern double autosteer_orig_altitude;
extern uint64_t autosteer_datetime;

uint16_t year=0; 
uint8_t month=0, day=0;
uint8_t hour=0, minute=0, seconds=0, hundredths=0;
double latitude=0;
double longitude=0;
double altitude=0;
double heading=0;
double heading90=0;
double roll=0;
bool got_pos=false;
bool got_attitude=false;

float last_heading = 0;
uint32_t last_heading_time = 0;

char nmea_buffer[160];
//char nmea_buffer1[160];
STINMEA sti(nmea_buffer, sizeof(nmea_buffer)); //handles PSTI
//SimpleNMEAParser gga(nmea_buffer1, sizeof(nmea_buffer1); //handles GGA

KFilter1 yawrate_filter(0.1, 1.0f, 0.0003f);
KFilter1 heading_filter(0.1, 1.0f, 0.0003f);
KFilter1 roll_filter(0.1, 1.0f, 0.0003f);

bool psti_process(char c) {
	if (sti.process(c)) {
		if (sti.getType() == 30) {
			uint8_t myear, mmonth, mday;
			uint8_t mhour, mminute, mseconds, mhundredths;

			//myear = sti.getYear();
			//mmonth = sti.getMonth();
			//mday = sti.getDay();
			mhour = sti.getHour();
			mminute = sti.getMinute();
			mseconds = sti.getSecond();
			mhundredths = sti.getHundredths();

			//if this messages has a different timestamp...
			if (/*year != myear ||
				month != mmonth ||
				day != mday ||*/
				hour != mhour ||
				minute != mminute ||
				seconds != mseconds ||
				hundredths != mhundredths) {
				// start a new message pair
				got_attitude=false;
				//year = myear;
				//month = mmonth;
				//day = mday;
				hour = mhour;
				minute = mminute;
				seconds = mseconds;
				hundredths = mhundredths;
			}

			got_pos = true;
			latitude = sti.getLatitude() / 10000000.0;
			longitude = sti.getLongitude() / 10000000.0;
			altitude = sti.getAltitude() / 1000.0;
		}

		if (sti.getType() == 36) {
			uint8_t myear, mmonth, mday;
			uint8_t mhour, mminute, mseconds, mhundredths;
			year = sti.getYear();
			month = sti.getMonth();
			day = sti.getDay();
			mhour = sti.getHour();
			mminute = sti.getMinute();
			mseconds = sti.getSecond();
			mhundredths = sti.getHundredths();

			//if this messages has a different timestamp...
			if (/*year != myear ||
				month != mmonth ||
				day != mday ||*/
				hour != mhour ||
				minute != mminute ||
				seconds != mseconds ||
				hundredths != mhundredths) {
				// start a new message pair
				got_pos=false;
				//year = myear;
				//month = mmonth;
				//day = mday;
				hour = mhour;
				minute = mminute;
				seconds = mseconds;
				hundredths = mhundredths;
			}

			got_attitude = true;
			//heading is 90 degress off of the dual antenna heading
			heading90 = sti.getHeading() / 100.0;
			//heading90 = heading_filter.filter(heading90);


			heading = (heading90 + 90);
			if (heading < 0)
				heading += 360;
			if (heading >= 360)
				heading -= 360;

			roll = sti.getPitch() / 100.0;
			//filter roll
			//roll = roll_filter.filter(roll);
		}
		if (got_pos && got_attitude) {
			double tilt_offset;
			double alt_offset1;
			double alt_offset2;
			double center_offset;
			float yaw_rate;

			float heading_delta;

			autosteer_datetime = ((float)seconds + hundredths /100.0) * 4;
			autosteer_datetime |= ((uint64_t)minute << 8);
			autosteer_datetime |= ((uint64_t)hour << 16);
			autosteer_datetime |= ((uint64_t)month << 24);
			autosteer_datetime |= (((uint64_t)day * 4) << 32);
			autosteer_datetime |= (((uint64_t)year - 1985) << 40);
			autosteer_datetime |= ((uint64_t)125 << 48); //zero
			autosteer_datetime |= ((uint64_t)125 << 56); //zero

			got_pos = false;
			got_attitude = false; //clear for next one

			autosteer_orig_lat = latitude;
			autosteer_orig_lon = longitude;
			autosteer_orig_altitude = altitude; 

			heading_delta = heading - last_heading;
			if (heading_delta > 180) heading_delta -= 360; //yawing to the left across 0
			else if (heading_delta <= -180) heading_delta += 360; //yawing to right across 0
			last_heading = heading;

			yaw_rate = heading_delta / 0.2;  //degrees/second
			//yaw_rate = yawrate_filter.filter(yaw_rate);
			autosteer_yawrate = yaw_rate;

			//offset from the tilt of the tractor
			tilt_offset = sin(roll * haversine::toRadians) * antenna_height;

			//account for distance from right antenna to center of tractor
			center_offset = cos(roll * haversine::toRadians) * antenna_right;

			//calculate ground-level elevation in center of tractor,
			//relative to right antenna
			alt_offset1 = cos(roll * haversine::toRadians) * antenna_height;
			alt_offset2 = sin(roll * haversine::toRadians) * center_offset;

			haversine::move_distance_bearing(latitude, longitude, (double)heading90,
											 tilt_offset + center_offset);

			altitude -= alt_offset1 - alt_offset2; //TODO: check me
			autosteer_altitude = altitude;

			if (antenna_forward) {
				//if gps is forward of axle, translate the position rearward
				haversine::move_distance_bearing(latitude, longitude, (double)heading, 
												 -antenna_forward);
			}

			if (sti.getMode() == 'R' || sti.getMode() == 'F') {
				autosteer_mode = sti.getMode();
				autosteer_lat = latitude;
				autosteer_lon = longitude;
				autosteer_roll = roll;
				autosteer_heading = heading;
				
				long north_speed = sti.getNorthVelocity();
				long east_speed = sti.getEastVelocity();

				//slow but faster than parsing GGA
				double speed = sqrt(north_speed * north_speed + east_speed * east_speed);
				autosteer_speed = speed * 3.6 / 1000.0; //convert to km/h

				return true;
			}
		}
	}

	//if (gga.process(c) {
	//
	//}

	return false;
}
