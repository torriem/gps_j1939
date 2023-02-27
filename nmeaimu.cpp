#include "nmeaimu.h"
#include "globals.h"
#include <elapsedMillis.h>
#include "NMEAParser.h"
#include "haversine.h"
#include "kfilter1.h"
#include "SimpleKalmanFilter.h"
#include <math.h>
#include "shared_nmea_buffer.h"
#include "nmea_checksum.h"
#include "imubase.h"
	
extern double autosteer_orig_lat;
extern double autosteer_orig_lon;
extern double autosteer_orig_altitude;

//minimum speed to use VTG to calculate IMU heading offset
#define MIN_VTG_SPEED 0.5 //kph
//minimum fix to fix distance to calculate a heading
#define MIN_FIX_DIST 3 //metre

static double last_lat = 400;
static double last_lon = 400;
static KFilter1 yawrate_filter(0.1, 1.0f, 0.0003f);
static KFilter1 heading_filter(0.1, 1.0f, 0.0003f);
static KFilter1 roll_filter(0.1, 1.0f, 0.0003f);

static FixHandler got_fix = NULL;
static SendNMEA send_nmea = NULL;

static NMEAParser<2> parser;

IMUBase *imu = NULL; //IMU to use
static bool  use_imu = false;

static bool got_gga = false;
static bool got_vtg = false;

static char buf[3];

static elapsedMillis nmea_timer;
static elapsedMillis imu_timer;
static elapsedMillis rate_timer;

extern bool imu_heading_offset_set;
extern float imu_heading_offset;
static float last_heading = 0;

//places to store NMEA fields
// GGA
static char fix_time[12];
static char latitude[15];
static char lat_ns[3];
static char longitude[15];
static char lon_ew[3];
static int fix_quality;
static char num_sats[4];
static char hdop[5];
static char altitude[12];
static char dgps_age[10];
static char geoid[10];

// VTG
static char vtg_heading[12] = { };
static char vtg_speed_knots[10] = { };
static float vtg_head;

static char checksum[5]; //* plus 2 byte checksum plus 2 CRLF bytes

static inline void generate_nmea(void) {
	/* generate new NMEA and VTG sentences based on
	 * our corrected position and heading
	 */

	char *vtg;

	double lat_min, lon_min;
	int lat1, lat2, lon1, lon2;

	lat_min = fabs(gps_latitude - int(gps_latitude)) * 60;
	lat1 = abs(int(gps_latitude)) * 100 + int(lat_min);
	lat2 = int((lat_min - int(lat_min)) * 10000000);

	lon_min = fabs(gps_longitude - int(gps_longitude)) * 60;
	lon1 = abs(int(gps_longitude)) * 100 + int(lon_min);
	lon2 = int((lon_min - int(lon_min)) * 10000000);

	//GGA
	snprintf(nmea_buffer,NMEA_BUFFER_SIZE,
	         "$GNGGA,%s,%d.%d,%s,%d.%d,%s,%d,%s,%s,%.2f,M,%s,M,%s,0000",
	         fix_time, 
		 lat1, lat2, (gps_latitude<0 ? "S" : "N"),
		 lon1, lon2, (gps_longitude<0 ? "W" : "E"),
		 fix_quality,
		 num_sats,
		 hdop,
		 gps_altitude,
		 geoid,
		 dgps_age);

	//add checksum and CRLF
	compute_nmea_checksum(nmea_buffer, checksum);
	strncat(nmea_buffer,checksum, NMEA_BUFFER_SIZE);
	strncat(nmea_buffer,"\r\n", NMEA_BUFFER_SIZE);

	//VTG
	vtg = nmea_buffer + strnlen(nmea_buffer, NMEA_BUFFER_SIZE);
	snprintf(vtg,NMEA_BUFFER_SIZE - strnlen(nmea_buffer, NMEA_BUFFER_SIZE),
	         "$GNVTG,%.3f,T,,M,%s,N,%.2f,K,D",
	         gps_heading,
		 vtg_speed_knots, //knots
		 gps_speed); //kph

	//add checksum and CRLF
	compute_nmea_checksum(vtg, checksum);
	strncat(nmea_buffer,checksum, NMEA_BUFFER_SIZE);
	strncat(nmea_buffer,"\r\n", NMEA_BUFFER_SIZE);

	//Send it
	send_nmea(nmea_buffer,strnlen(nmea_buffer, NMEA_BUFFER_SIZE));

}

static inline void process_imu(void) {
	float heading_delta;
	double roll_rad;
	double yaw;
	double heading90;
	double tilt_offset;
	double center_offset;
	double new_offset;
	float roll_direction;

	got_gga = false;
	got_vtg = false;

	if (imu_reverse) 
		roll_direction = -1;
	else 
		roll_direction = 1;

	//look at older IMU position
	if (imu)
		yaw = imu->get_yaw(imu_lookback);

	gps_orig_latitude = gps_latitude;
	gps_orig_longitude = gps_longitude;

	if (use_imu && yaw < 400) {
		//if we have a valid IMU reading, calculate the IMU
		//heading offset and do roll compensation.
		
		gps_roll = imu->get_roll(imu_lookback); //already offset

		if(!imu_heading_offset_set && gps_speed <= MIN_VTG_SPEED) {
			//calculate heading based on fix to fix
			//use that to figure out IMU heading offset
			if (last_lat < 400) {
				if (haversine::distance(last_lat, last_lon,
				                        gps_latitude,
						        gps_longitude) > MIN_FIX_DIST) {
					gps_heading = haversine::bearing(
					                         last_lat, last_lon,
							         gps_latitude, gps_longitude);
					imu_heading_offset = gps_heading - yaw;
					if (imu_heading_offset < 0) imu_heading_offset += 360;
					imu_heading_offset_set = true;
				}

			} else {
				last_lat = gps_latitude;
				last_lon = gps_longitude;
			}
		} else if (gps_speed > MIN_VTG_SPEED) {
			//if we're moving, calculate an IMU to real heading offset
			new_offset = vtg_head - yaw;
			if (new_offset < 0) new_offset += 360;

			if (!imu_heading_offset_set) {
				imu_heading_offset_set = true;
				imu_heading_offset  = new_offset;
			} else {
				if (fabs(new_offset - imu_heading_offset) > 150) {
					//probably driving in reverse
					new_offset -= 180;
					if (new_offset < 0) new_offset += 360;
				}
			}

			//tweak heading offset based on latest VTG heading
			imu_heading_offset = 0.7*imu_heading_offset + 0.3*new_offset;
		}		

		if(imu_heading_offset_set) {
			//Use the IMU heading instead of the pure VTG heading
			gps_heading = fmod(yaw + imu_heading_offset,360);
			if (gps_heading < 0) gps_heading += 360;
		} else {
			//no offset yet calculated, so throw in the VTG heading
			gps_heading = vtg_head;
		}

		roll_rad = gps_roll * M_PI / 180.0;
	} else {
		//we either don't have an IMU or we haven't read it yet
		//so don't do roll compensation, but do adjust the GPS
		//as best we can to the center of the tractor from its
		//potentially offset position.
		
		roll_rad = 0;
		gps_heading = vtg_head;
		gps_roll = 0;
	}

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
	generate_nmea();
}

void GGA_handler() {
	if (nmea_timer > 80) {
		//if at least 80 ms has elapsed since the last 
		//NMEA message, we need to look again for VTG
		got_vtg = false;
		nmea_timer = 0; //??
	}

	got_gga = true;

	//TODO: snag copies of unmodified fields for re-sending GGA later
	
	parser.getArg(0,fix_time);
	uint8_t hours, minutes, seconds, hundredths;

	buf[0] = fix_time[0];
	buf[1] = fix_time[1];
	hours = atoi(buf);

	buf[0] = fix_time[2];
	buf[1] = fix_time[3];
	minutes = atoi(buf);

	buf[0] = fix_time[4];
	buf[1] = fix_time[5];
	seconds = atoi(buf);

	if(fix_time[6] == '.' && fix_time[7] != 0 && fix_time[8] != 0) {
		buf[0] = fix_time[7];
		buf[1] = fix_time[8];
		hundredths = atoi(buf);
	} else {
		hundredths = 0;
	}

	gps_j1939_datetime = ((float)seconds + hundredths /100.0) * 4;
	gps_j1939_datetime |= ((uint64_t)minutes << 8);
	gps_j1939_datetime |= ((uint64_t)hours << 16);
	//Date not part of GGA, so fake it for now. 1 Jan 1985
	gps_j1939_datetime |= 0x7d7d000401000000;
	
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

	parser.getArg(5, fix_quality);
	gps_mode = fix_quality;

	if (gps_mode == 0) {
		//No GPS position at all, so ignore
		got_gga = false;
		got_vtg = false;
		return;
	}

	parser.getArg(8, altitude);
	gps_altitude = atof(altitude);

	parser.getArg(6, num_sats);
	parser.getArg(7, hdop);
	parser.getArg(10, geoid);
	parser.getArg(12, dgps_age);

	if (got_vtg) {
		//we've got our pair now, we can do
		//imu processing and send the data
		//back to the main program
		process_imu();
	}
}

void VTG_handler() {
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

void error_handler() {

}


//TODO: pass in handler to call when we've got a new position and
//need to send it over CAN.
void setup_nmea_parser(FixHandler fix_handler, IMUBase *the_imu = NULL, SendNMEA send_nmea_handler = NULL) {
	parser.setErrorHandler(error_handler);
	parser.addHandler("G-GGA", GGA_handler);
	parser.addHandler("G-VTG", VTG_handler);

	got_fix = fix_handler;
	send_nmea = send_nmea_handler;

	imu_heading_offset_set = false;
	imu = the_imu;

	//initialize our timeout counters
	nmea_timer = 0;
	rate_timer = 0;

	last_lat = 400;
	last_lon = 400;
	buf[2] = 0;


	if (imu) {
		use_imu = true;
	}
}

void nmea_process(char c) {
	parser << c;
}
