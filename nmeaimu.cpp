#include "nmeaimu.h"
#include "globals.h"
#include "defaults.h"
#include "whichteensy.h"
#ifndef TEENSY
#include <elapsedMillis.h>
#endif
#include "NMEAParser.h"
#include "haversine.h"
//#include "kfilter1.h"
//#include "SimpleKalmanFilter.h"
#include <math.h>
#include "shared_nmea_buffer.h"
#include "imubase.h"
	
static double last_lat = 400;
static double last_lon = 400;
//static KFilter1 yawrate_filter(0.1, 1.0f, 0.0003f);
//static KFilter1 heading_filter(0.1, 1.0f, 0.0003f);
//static KFilter1 roll_filter(0.1, 1.0f, 0.0003f);

static NMEAParser<3> parser;

static FixHandler got_fix = NULL;
static IMUBase *imu = NULL; //IMU to use

static bool  use_imu = false;

static bool got_gga = false;
static bool got_vtg = false;

static char buf[3];

static elapsedMillis nmea_timer;
static elapsedMillis rate_timer;

static float last_heading = 0;

//places to store NMEA fields
// GGA
static char latitude[15];
static char lat_ns[3];
static char longitude[15];
static char lon_ew[3];
static char altitude[12];

// VTG
static char vtg_heading[12] = { };
static char vtg_speed_knots[10] = { };
static float vtg_head;

static inline void process_imu(void) {
	float heading_delta;
	double roll_rad;
	double heading90;
	double tilt_offset;
	double center_offset;
	double new_offset;

	got_gga = false;
	got_vtg = false;

	gps_orig_latitude = gps_latitude;
	gps_orig_longitude = gps_longitude;
	gps_orig_heading = vtg_head;

	if (imu_current_yaw < 400) {
		//if we have a valid IMU reading, calculate the IMU
		//heading offset and do roll compensation.
		
		if(imu_heading_offset > 360 && gps_speed <= MIN_VTG_SPEED) {
			//If no imu offset is known, and we're going too slow,
			//calculate heading based on fix to fix
			//use that to figure out IMU heading offset
			if (last_lat < 400) {
				if (haversine::distance(last_lat, last_lon,
				                        gps_latitude,
						        gps_longitude) > MIN_FIX_DIST) {
					gps_heading = haversine::bearing_degrees(
					                         last_lat, last_lon,
							         gps_latitude, gps_longitude);
					imu_heading_offset = gps_heading - imu_current_yaw;
					if (new_offset > 180) new_offset -= 360;
					if (new_offset < -180) new_offset += 360;
					//if (imu_heading_offset < 0) imu_heading_offset += 360;
					/*
					Serial.print("heading offset set from fix to fix: ");
					Serial.print(gps_heading);
					Serial.print("-");
					Serial.print(imu_current_yaw);
					Serial.print("=");
					Serial.println(imu_heading_offset);
					*/
				}

			} else {
				last_lat = gps_latitude;
				last_lon = gps_longitude;
			}
		} else if (gps_speed > MIN_VTG_SPEED) {
			//if we're moving, calculate an IMU to real heading offset

			//if vtg heading has changed dramatically, but if the raw imu yaw has
			//not changed significantly, we must be reversing.

			new_offset = vtg_head - imu_current_yaw;
			if (new_offset > 180) new_offset -= 360;
			if (new_offset < -180) new_offset += 360;

			if (imu_heading_offset > 360) {
				//if heading offset is not already set, we need to set it here.
				//hopefully we're going forward at this time.  If not, what to do
				//TODO: figure out better reverse logic
				imu_heading_offset  = new_offset;
				Serial.print("heading offset set from vtg: ");
				Serial.print(gps_heading);
				Serial.print("-");
				Serial.print(imu_current_yaw);
				Serial.print("=");
				Serial.println(imu_heading_offset);
			} else {
				//heading offset is set, let's refine it.
				if (fabs(new_offset - imu_heading_offset) > 150) {
					Serial.println("Reversed.");
					//probably driving in reverse
					new_offset -= 180;
					if (new_offset < 0) new_offset += 360;
				}
			}

			//tweak heading offset based on latest VTG heading
			imu_heading_offset = 0.7*imu_heading_offset + 0.3*new_offset;
		}		

		if(imu_heading_offset < 400) {
			//Use the IMU heading instead of the pure VTG heading
			gps_heading = fmod(imu_current_yaw + imu_heading_offset,360);
			if (gps_heading < 0) gps_heading += 360;
		} else {
			//no offset yet calculated, so throw in the VTG heading
			gps_heading = vtg_head;
		}

	} else {
		//no valid IMU heading numbers yet.
		gps_heading = vtg_head;

		if (gps_roll > 360) {
			//if no valid IMU roll numbers, do without terrain
			//compensation, although any antenna offset adjustments
			//will be wildly incorrect.
			roll_rad = 0;
			gps_roll = 0;
		}
	}

	roll_rad = gps_roll * M_PI / 180.0;

	//do roll compensation and adjust lat and lon for the 
	//offsets of the GPS unit

	heading90 = gps_heading - 90;
	if (heading90 <0) heading90 +=360;

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

static void GGA_handler() {
	if (nmea_timer > 80) {
		//if at least 80 ms has elapsed since the last 
		//NMEA message, we need to look again for VTG
		got_vtg = false;
		nmea_timer = 0; //??
	}
	gps_roll = imu_current_roll;

	got_gga = true;

	//TODO: snag copies of unmodified fields for re-sending GGA later
	
	parser.getArg(0,gps_fix_time);
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
	//Date comes in from RMC if present, otherwise we use a default
	//date set in the setup function.
	gps_j1939_datetime |= ((uint64_t)month << 24);
	gps_j1939_datetime |= (((uint64_t)day * 4) << 32);
	gps_j1939_datetime |= (((uint64_t)year - 1985) << 40);
	gps_j1939_datetime |= ((uint64_t)125 << 48); //zero
	gps_j1939_datetime |= ((uint64_t)125 << 56); //zero

	//gps_j1939_datetime |= 0x7d7d000401000000;
	
	parser.getArg(1,latitude);
	parser.getArg(2,lat_ns);

	gps_latitude = atof(latitude);
	gps_latitude = int(gps_latitude / 100) /*deg part*/ +
		          (gps_latitude - int(gps_latitude/100) * 100) / 60; /*decimal part*/

	if (lat_ns[0] == 'S') 
		gps_latitude = -gps_latitude;

	parser.getArg(3,longitude);
	parser.getArg(4,lon_ew);

	gps_longitude = atof(longitude);
	gps_longitude = int(gps_longitude / 100) /*deg part*/ +
		          (gps_longitude - int(gps_longitude/100) * 100) / 60; /*decimal part*/
	
	if (lon_ew[0] == 'W')
		gps_longitude = -gps_longitude;

	parser.getArg(5, gps_mode);

	if (gps_mode == 0) {
		//No GPS position at all, so ignore
		got_gga = false;
		got_vtg = false;
		return;
	}

	parser.getArg(8, altitude);
	gps_altitude = atof(altitude);

	parser.getArg(6, gps_num_sats);
	parser.getArg(7, gps_hdop);
	parser.getArg(10, gps_geoid);
	parser.getArg(12, gps_dgps_age);

	if (got_vtg) {
		//we've got our pair now, we can do
		//imu processing and send the data
		//back to the main program
		process_imu();
	}
}

static void VTG_handler() {
	if (nmea_timer > 80) {
		//if at least 80 ms has elapsed since the last 
		//NMEA message, we need to look again for GGA
		got_gga = false;
		nmea_timer = 0;
	}

	got_vtg = true;

	// vtg heading
	parser.getArg(0, vtg_heading);

	if (vtg_heading[0] != 0) {
		vtg_head = atof(vtg_heading);
	} else {
		vtg_head = 0; //unknown cause we're prob standing still
	}

	// vtg Speed knots
	parser.getArg(4, vtg_speed_knots);
	gps_speed = atof(vtg_speed_knots);
	gps_speed *= 1.852; //convert to kph

	if(got_gga) {
		process_imu();
	}

}

static void RMC_handler() {
	parser.getArg(8,gps_fix_date);
}

static void error_handler() {

}

unsigned long last_time = 0;

static void GGA_pre_handler() {
	unsigned long t = millis();
	Serial.println(t - last_time);
	last_time = t;
}

namespace nmea_imu {
	void setup(FixHandler fix_handler, IMUBase *the_imu) {
		parser.setErrorHandler(error_handler);
		parser.addHandler("G-GGA", GGA_handler);
		parser.addHandler("G-VTG", VTG_handler);
		parser.addHandler("G-RMC", RMC_handler);

		got_fix = fix_handler;

		imu_heading_offset_set = false;
		imu = the_imu;

		//initialize our timeout counters
		nmea_timer = 0;
		rate_timer = 0;

		last_lat = 400;
		last_lon = 400;
		buf[2] = 0;

		//Date not part of GGA, so fake it if no RMC messages. 1 Jan 1985
		strcpy(gps_fix_date, "010185");

		if (imu) {
			use_imu = true;
		}
	}

	void process_byte(char c) {
		parser << c;
	}

	void set_imu(IMUBase *the_imu) {
		imu = the_imu;
	}

	void set_on_fix_handler(FixHandler fix_handler) {
		got_fix = fix_handler;
	}
}
