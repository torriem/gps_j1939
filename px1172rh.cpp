#include "px1172rh.h"
#include "globals.h"
#include "shared_nmea_buffer.h"
#include "NMEAParser.h"
#include "stinmea.h"
#include "haversine.h"
#include "imubase.h"
//#include "kfilter1.h"
//#include "SimpleKalmanFilter.h"
#include "conversions.h"
#include "math.h"
#include "whichteensy.h"
#ifndef TEENSY
#include <elapsedMillis.h>
#endif
	
static bool got_pos=false;

static bool got_good_attitude = false;
static bool got_attitude=false;

static float last_heading = 0;
static uint32_t last_heading_time = 0;

static char sti_nmea_buffer[160];
static STINMEA sti(sti_nmea_buffer, sizeof(sti_nmea_buffer)); //handles PSTI

static NMEAParser<2> parser;

char buf[3];
static char latitude[15];
static char lat_ns[3];
static char longitude[15];
static char lon_ew[3];
static char altitude[12];

static elapsedMillis rate_timer;
static elapsedMillis nmea_timer;


//static KFilter1 yawrate_filter(0.1, 1.0f, 0.0003f);
//static KFilter1 heading_filter(0.1, 1.0f, 0.0003f);
//static KFilter1 roll_filter(0.1, 1.0f, 0.0003f);

static FixHandler got_fix = NULL;
static IMUBase *imu = NULL;


static inline void process_imu(void) {
	float heading_delta;
	double roll_rad;
	double heading90;
	double tilt_offset;
	double center_offset;

	gps_orig_latitude = gps_latitude;
	gps_orig_longitude = gps_longitude;

	//Perform terrain compensation.  If an IMU is present, we can combine
	//the IMU roll reading with the dual GPS roll reading perhaps.
	
	//do roll compensation and adjust lat and lon for the 
	//offsets of the GPS unit

	roll_rad = RADIANS(gps_roll);

	heading90 = gps_heading - 90;
	if (heading90 <0) heading90 += 360;

	//lateral offset from tilt of tractor
	tilt_offset = sin(roll_rad) * antenna_height;
	//lateral offset from antenna to center of tractor
	center_offset = cos(roll_rad) * antenna_right;

	//move the gps position
	haversine::move_distance_bearing(gps_latitude,
					 gps_longitude,
					 heading90,
					 tilt_offset + center_offset);

	//Adjust altitude
	gps_orig_altitude = gps_altitude;

	gps_altitude -= cos(roll_rad) * antenna_height +
			      sin(roll_rad) * center_offset;

	if (antenna_forward) {
		haversine::move_distance_bearing(gps_latitude,
						 gps_longitude,
						 gps_heading,
						 -antenna_forward);
	}

	//calculate yaw rate.
	heading_delta = gps_heading - last_heading;
	if (heading_delta > 180) heading_delta -= 360; //yawing left across 0
	else if (heading_delta <= -180) heading_delta += 360; //yawing right across 0

	last_heading = gps_heading;
	gps_yawrate = heading_delta * 1000.0 / rate_timer;

	rate_timer = 0;  //should approximate time between messages

	if(got_fix) got_fix();
}

static void PSTI_handler() {
	int pstinum = 0;
	float east_vel;
	float north_vel;
	char fix_type[6];
	char dual_mode[6];
	float heading90;
	char temp[10];

	heading90 = 0;
	parser.getArg(0,pstinum);
	
	if(pstinum == 30) {
		if (nmea_timer > 80) {
			//if at least 80 ms has elapsed since the last 
			//NMEA message, we need to look again for VTG
			got_attitude = false;
			nmea_timer = 0; //??
		}
		got_pos = true;

		parser.getArg(12, fix_type);
		switch (fix_type[0]) {
		case 'R':
			gps_mode = 4;
			break;
		case 'F':
			gps_mode = 5;
			break;
		case 'N':
			gps_mode = 0;
			got_attitude = false;
			got_pos = false;
			return; //there's no fix here?
		case 'D':
			gps_mode = 2;
			break;

		default:
			gps_mode = 1;
		}
		
		parser.getArg(1, gps_fix_time);

		parser.getArg(3, latitude);
		parser.getArg(4, lat_ns);

		gps_latitude = atof(latitude);
		gps_latitude = int(gps_latitude / 100) /*deg part*/ +
			          (gps_latitude - int(gps_latitude/100) * 100) / 60; /*decimal part*/

		if (lat_ns[0] == 'S') 
			gps_latitude = -gps_latitude;

		parser.getArg(5, longitude);
		parser.getArg(6, lon_ew);

		gps_longitude = atof(longitude);
		gps_longitude = int(gps_longitude / 100) /*deg part*/ +
				  (gps_longitude - int(gps_longitude/100) * 100) / 60; /*decimal part*/
		
		if (lon_ew[0] == 'W')
			gps_longitude = -gps_longitude;

		parser.getArg(7, gps_altitude);

		//velocity vectors in m/s, in case we don't
		//have dual GPS and no IMU
		parser.getArg(8, east_vel);
		parser.getArg(9, north_vel);

		parser.getArg(11,gps_fix_date);
		parser.getArg(13,gps_dgps_age);



		if (! (got_attitude && got_good_attitude)) {
			//if 036 was seen but dual gps wasn't ready or had 
			//good enough signal
			//or if 036 has not been seen yet,
			//We will set gps_heading to the velocity direction.

			gps_heading = RADIANS(atan2(east_vel, north_vel));
		}

	}

	if (pstinum == 36) {
		if (nmea_timer > 80) {
			//if at least 80 ms has elapsed since the last 
			//NMEA message, we need to look again for VTG
			got_pos = false;
			nmea_timer = 0; //??
		}
		got_attitude = true;

		parser.getArg(6,dual_mode);
		Serial.println(dual_mode);
		if (dual_mode[0] != 'R' && dual_mode[0] != 'F') {
			//we have insufficient fix quality to do dual GPS
			//or there's no second antenna plugged in.

			got_good_attitude = false;
			gps_roll = 0.0;
			//heading will yet be or is already set by the PSTI,030 
			//parser above.
		} else {
			got_good_attitude = true;

			parser.getArg(3, heading90);
			parser.getArg(4, gps_roll);

			gps_heading = heading90 + 90;
			if (gps_heading >= 360)
				gps_heading -= 360;
			if (gps_heading <0 )
				gps_heading += 360;
		}
	}

	if (got_pos && got_attitude) {
		//TODO: parse fix time and fix date
		uint8_t hours, minutes, seconds, hundredths;
		uint8_t month, day, year;

		buf[2] = 0;

		buf[0] = gps_fix_time[0];
		buf[1] = gps_fix_time[1];
		hours = atoi(buf);

		buf[0] = gps_fix_time[2];
		buf[1] = gps_fix_time[3];
		minutes = atoi(buf);

		buf[0] = gps_fix_time[4];
		buf[1] = gps_fix_time[5];
		seconds = atoi(buf);

		if(gps_fix_time[6] == '.' && gps_fix_time[7] != 0 && gps_fix_time[8] != 0) {
			buf[0] = gps_fix_time[7];
			buf[1] = gps_fix_time[8];
			hundredths = atoi(buf);
		} else {
			hundredths = 0;
		}

		buf[0] = gps_fix_date[0];
		buf[1] = gps_fix_date[1];
		day = atoi(buf);

		buf[0] = gps_fix_date[2];
		buf[1] = gps_fix_date[3];
		month = atoi(buf);

		buf[0] = gps_fix_date[4];
		buf[1] = gps_fix_date[5];
		year = atoi(buf);

		gps_j1939_datetime = ((float)seconds + hundredths /100.0) * 4;
		gps_j1939_datetime |= ((uint64_t)minutes << 8);
		gps_j1939_datetime |= ((uint64_t)hours << 16);
		gps_j1939_datetime |= ((uint64_t)month << 24);
		gps_j1939_datetime |= (((uint64_t)day * 4) << 32);
		gps_j1939_datetime |= (((uint64_t)year - 1985) << 40);
		gps_j1939_datetime |= ((uint64_t)125 << 48); //zero
		gps_j1939_datetime |= ((uint64_t)125 << 56); //zero

		got_pos = false;
		got_attitude = false; //clear for next one

		process_imu();
	}
}

static void GGA_handler() {
	parser.getArg(6, gps_num_sats);
	parser.getArg(7, gps_hdop);
	parser.getArg(10, gps_geoid);
	parser.getArg(12, gps_dgps_age);

}

static void error_handler() {

}

namespace px1172rh {
	void setup(FixHandler fix_handler){
		parser.setErrorHandler(error_handler);
		parser.addHandler("PSTI", PSTI_handler);
		parser.addHandler("G-GGA", GGA_handler);

		got_pos = false;
		got_attitude = false;
		buf[2] = 0;

		got_fix = fix_handler;
	}

	void set_on_fix_handler(FixHandler fix_handler) {
		got_fix = fix_handler;
	}

	void process_byte(char c) {
		parser << c;
	}
}


#if 0
void psti_process(char c) {
	if (sti.process(c)) {
		if (sti.getType() == 30) {
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
			fix30_mode = sti.getMode();
		}

		if (sti.getType() == 36) {
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

		long north_speed = sti.getNorthVelocity();
		long east_speed = sti.getEastVelocity();

		//slow but faster than parsing GGA
		double speed = sqrt(north_speed * north_speed + east_speed * east_speed);
		gps_speed = speed * 3.6 / 1000.0; //convert to km/h


		if (got_pos && got_attitude) {
			double tilt_offset;
			double alt_offset1;
			double alt_offset2;
			double center_offset;
			float yaw_rate;

			float heading_delta;

			gps_j1939_datetime = ((float)seconds + hundredths /100.0) * 4;
			gps_j1939_datetime |= ((uint64_t)minute << 8);
			gps_j1939_datetime |= ((uint64_t)hour << 16);
			gps_j1939_datetime |= ((uint64_t)month << 24);
			gps_j1939_datetime |= (((uint64_t)day * 4) << 32);
			gps_j1939_datetime |= (((uint64_t)year - 1985) << 40);
			gps_j1939_datetime |= ((uint64_t)125 << 48); //zero
			gps_j1939_datetime |= ((uint64_t)125 << 56); //zero

			got_pos = false;
			got_attitude = false; //clear for next one

			gps_orig_latitude = latitude;
			gps_orig_latitude = longitude;
			gps_orig_altitude = altitude; 
			//unless both receives have RTK or Float, we can't really do TCM
			//so this will only return R or F if STI,036 shows R or F.
			char autosteer_mode = sti.getMode();
			switch(autosteer_mode) {
			case 'R':
				gps_mode = 4;
				break;
			case 'F':
				gps_mode = 5;
				break;
			case 'D':
				gps_mode = 2;
				break;
			default:
				gps_mode = 0;
			}

			if (fix30_mode != 'N' && sti.getMode() != 'R' && sti.getMode() != 'F') {
				//low-quality GPS fix, no STI,036 information
				//calculate approximate heading from STI,030 velocity vectors
				//assume a roll of zero (no terrain compensation)
				//translate the GPS position to the center only
				//not really sure why I am doing this since you can't steer
				//with this level of GPS. Mapping maybe?

				heading = atan2(east_speed, north_speed) * 180.0 / PI;
				if (heading <0) heading += 360;

				gps_heading = heading;
				gps_roll = 0;

				heading_delta = heading - last_heading;
				if (heading_delta > 180) heading_delta -= 360; //yawing to the left across 0
				else if (heading_delta <= -180) heading_delta += 360; //yawing to right across 0
				last_heading = heading;

				//TODO make work for different message rates other than 5 Hz
				yaw_rate = heading_delta / 0.2;  //degrees/second
				//yaw_rate = yawrate_filter.filter(yaw_rate);
				gps_yawrate = yaw_rate;

				//account for distance from right antenna to center of tractor
				center_offset = cos(roll * haversine::toRadians) * antenna_right;
				haversine::move_distance_bearing(latitude, longitude, (double)heading90,
				                                 center_offset);

				gps_altitude = altitude - antenna_height;
				gps_latitude = latitude;
				gps_longitude = longitude;

				if(got_psti_fix)
					got_psti_fix();

			} else if (sti.getMode() == 'R' || sti.getMode() == 'F') {
				//if RTK, do terrain compensation
				heading_delta = heading - last_heading;
				if (heading_delta > 180) heading_delta -= 360; //yawing to the left across 0
				else if (heading_delta <= -180) heading_delta += 360; //yawing to right across 0
				last_heading = heading;

				//TODO make work for different message rates other than 5 Hz
				yaw_rate = heading_delta / 0.2;  //degrees/second
				//yaw_rate = yawrate_filter.filter(yaw_rate);
				gps_yawrate = yaw_rate;

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
				gps_altitude = altitude;

				if (antenna_forward) {
					//if gps is forward of axle, translate the position rearward
					haversine::move_distance_bearing(latitude, longitude, (double)heading,
					                                 -antenna_forward);
				}

				gps_latitude = latitude;
				gps_longitude = longitude;
				gps_roll = roll;
				gps_heading = heading;
				
				if(got_psti_fix)
					got_psti_fix();
			}
		}
	}
}
#endif
