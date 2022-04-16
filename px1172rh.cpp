#include "px1172rh.h"
#include "stinmea.h"
#include "haversine.h"
	
uint8_t year=0, month=0, day=0;
uint8_t hour=0, minute=0, seconds=0, hundredths=0;
long latitude=0;
long longitude=0;
long altitude=0;
long pitch=0;
long heading=0;
long heading90=0;
double lat=0;
double lon=0;
bool got_pos=false;
bool got_attitude=false;

char nmea_buffer[160];
STINMEA sti(nmea_buffer, sizeof(nmea_buffer));

bool psti_process(char c) {
	if (sti.process(c)) {
		if (sti.getType() == 30) {
			uint8_t myear, mmonth, mday;
			uint8_t mhour, mminute, mseconds, mhundredths;

			myear = sti.getYear();
			mmonth = sti.getMonth();
			mday = sti.getDay();
			mhour = sti.getHour();
			mminute = sti.getMinute();
			mseconds = sti.getSecond();
			mhundredths = sti.getHundredths();

			//if this messages has a different timestamp...
			if (year != myear ||
				month != mmonth ||
				day != mday ||
				hour != mhour ||
				minute != mminute ||
				seconds != mseconds ||
				hundredths != mhundredths) {
				// start a new message pair
				got_attitude=false;
				year = myear;
				month = mmonth;
				day = mday;
				hour = mhour;
				minute = mminute;
				seconds = mseconds;
				hundredths = mhundredths;
			}

			got_pos = true;
			latitude = sti.getLatitude();
			longitude = sti.getLongitude();
			altitude = sti.getAltitude();
		}

		if (sti.getType() == 36) {
			uint8_t myear, mmonth, mday;
			uint8_t mhour, mminute, mseconds, mhundredths;
			myear = sti.getYear();
			mmonth = sti.getMonth();
			mday = sti.getDay();
			mhour = sti.getHour();
			mminute = sti.getMinute();
			mseconds = sti.getSecond();
			mhundredths = sti.getHundredths();

			//if this messages has a different timestamp...
			if (year != myear ||
				month != mmonth ||
				day != mday ||
				hour != mhour ||
				minute != mminute ||
				seconds != mseconds ||
				hundredths != mhundredths) {
				// start a new message pair
				got_pos=false;
				year = myear;
				month = mmonth;
				day = mday;
				hour = mhour;
				minute = mminute;
				seconds = mseconds;
				hundredths = mhundredths;
			}

			got_attitude = true;
			//heading is 90 degress off of the dual antenna heading
			heading90 = sti.getHeading();
			heading = (heading90 - 9000);
			if (heading < 0)
				heading += 36000;
			//heading = sti.getHeading();
			pitch = sti.getPitch();
		}
		if (got_pos && got_attitude) {
			double tilt_offset;
			double alt_offset1;
			double alt_offset2;
			double center_offset;
			double roll;
			//we need to do some trigonometry so unpack the values
			//to doubles
			lat = latitude / 10000000.0;
			lon = longitude / 10000000.0;
			roll = pitch / 100.0;

			//offset from the tilt of the tractor
			tilt_offset = sin(roll * haversine::toRadians) * antenna_height;

			//account for distance from left antenna to center of tractor
			center_offset = cos(roll * haversine::toRadians) * antenna_left;

			//calculate ground-level elevation in center of tractor,
			//relative to left antenna
			alt_offset1 = cos(roll * haversine::toRadians) * antenna_height;
			alt_offset2 = sin(roll * haversine::toRadians) * center_offset;

			haversine::move_distance_bearing(lat, lon, (double)heading90 / 100.0,
											 tilt_offset + center_offset);

			altitude -= ((alt_offset1 - alt_offset2) * 1000.0);
			autosteer_altitude = altitude;

			if (antenna_forward) {
				//if gps is forward of axle, translate the position rearward
				haversine::move_distance_bearing(lat, lon, (double)heading / 100.0, 
												 -antenna_forward);
			}

			if (sti.getMode() == 'R' || sti.getMode() == 'F') {
				autosteer_mode = sti.getMode();
				autosteer_lat = lat;
				autosteer_lon = lon;
				autosteer_roll = roll;
				autosteer_heading = heading;
				
				long north_speed = sti.getNorthVelocity();
				long east_speed = sti.getEastVelocity();

				//slow but faster than parsing GGA
				double speed = sqrt(north_speed * north_speed + east_speed * east_speed);
				autosteer_speed = speed / 16666.67; //convert to km/h

				return true;
			}
		}
	}

	return false;
}
