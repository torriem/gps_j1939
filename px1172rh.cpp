#include "px1172rh.h"
#include "stinmea.h"
#include "haversine.h"
	
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

float last_heading = 0;
uint32_t last_heading_time = 0;

char nmea_buffer[160];
//char nmea_buffer1[160];
STINMEA sti(nmea_buffer, sizeof(nmea_buffer)); //handles PSTI
//SimpleNMEAParser gga(nmea_buffer1, sizeof(nmea_buffer1); //handles GGA

float yaw_P = 1.0f;
float yaw_varProcess = 0.0003f;
float yaw_Xe = 0;

float heading_P = 1.0f;
float heading_varProcess = 0.0003f;
float heading_Xe = 0;

float roll_P = 1.0f;
float roll_varProcess = 0.0003f;
float roll_Xe = 0;

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

			/*
			heading_Pc = heading_P + heading_varProcess;
			heading_G = heading_Pc / (heading_Pc / heading90);
			heading_P = (1 - heading_G) * heading_Pc;
			heading_Xp = heading_Xe;
			heading_Zp = heading_Xp;
			heading_Xe = (heading_G * (yaw_rate - heading_Zp)) + heading_Xp;
			heading90 = heading_Xe; //hopefully filtered heading
			*/

			heading = (heading90 + 9000);
			if (heading < 0)
				heading += 36000;
			if (heading >= 36000)
				heading -= 36000;

			//heading = sti.getHeading();
			pitch = sti.getPitch();
		}
		if (got_pos && got_attitude) {
			double tilt_offset;
			double alt_offset1;
			double alt_offset2;
			double center_offset;
			double roll;
			float yaw_rate;

			float roll_Pc, roll_G, roll_Xp, roll_Zp; //for filtering yaw rate
			float yaw_Pc, yaw_G, yaw_Xp, yaw_Zp; //for filtering yaw rate
			float heading_delta;

			//we need to do some trigonometry so unpack the values
			//to doubles
			lat = latitude / 10000000.0;
			lon = longitude / 10000000.0;
			roll = -pitch / 100.0;

			//filter roll
			roll_Pc = roll_P + roll_varProcess;
			roll_G = roll_Pc / (roll_Pc / roll);
			roll_P = (1 - roll_G) * roll_Pc;
			roll_Xp = roll_Xe;
			roll_Zp = roll_Xp;
			roll_Xe = (roll_G * (roll - roll_Zp)) + roll_Xp;
			roll = roll_Xe; //hopefully this is right

			heading_delta = heading - last_heading;
			if (heading_delta > 180) heading_delta -= 360; //yawing to the left across 0
			else if (heading_delta <= -180) heading_delta += 360; //yawing to right across 0
			yaw_rate = heading_delta / ((last_heading_time - millis()) * 1000); //degrees/second
			last_heading_time = millis();
			yaw_Pc = yaw_P + yaw_varProcess;
			yaw_G = yaw_Pc / (yaw_Pc / roll);
			yaw_P = (1 - yaw_G) * yaw_Pc;
			yaw_Xp = yaw_Xe;
			yaw_Zp = yaw_Xp;
			yaw_Xe = (yaw_G * (yaw_rate - yaw_Zp)) + yaw_Xp;
			autosteer_yawrate = yaw_Xe; //hopefully filtered yaw rate to send back to main program

			//offset from the tilt of the tractor
			tilt_offset = sin(roll * haversine::toRadians) * antenna_height;

			//account for distance from left antenna to center of tractor
			center_offset = cos(roll * haversine::toRadians) * antenna_right;

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

	//if (gga.process(c) {
	//
	//}

	return false;
}
