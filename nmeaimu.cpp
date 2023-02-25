#include "nmeaimu.h"
#include "Adafruit_BNO08x_RVC.h"
#include "elapsedMillis.h"
#include "NMEAParser.h"
#include "haversine.h"
#include "kfilter1.h"
#include "SimpleKalmanFilter.h"
#include <math.h>
#include "shared_nmea_buffer.h"
#include "nmea_checksum.h"
	
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

//minimum speed to use VTG to calculate IMU heading offset
#define MIN_VTG_SPEED 0.5 //kph
//minimum fix to fix distance to calculate a heading
#define MIN_FIX_DIST 1 //metre

extern bool imu_use_pitch;
extern float imu_roll_offset;
extern bool imu_reverse;

static double last_lat = 400;
static double last_lon = 400;
static KFilter1 yawrate_filter(0.1, 1.0f, 0.0003f);
static KFilter1 heading_filter(0.1, 1.0f, 0.0003f);
static KFilter1 roll_filter(0.1, 1.0f, 0.0003f);

static FixHandler got_fix = NULL;
static SendNMEA send_nmea = NULL;

static NMEAParser<2> parser;

//Use adafruit's library or AOG's modified library?
#define MAX_LOOKBACK 20
static Adafruit_BNO08x_RVC rvc;
static BNO08x_RVC_Data rvc_data[MAX_LOOKBACK];
static int16_t current_rvc = 0;
static bool  use_imu = false;

static bool got_gga = false;
static bool got_vtg = false;

static char buf[3];

//how far back to look at IMU
static uint16_t imu_lookback = 11;

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

	lat_min = fabs(autosteer_lat - int(autosteer_lat)) * 60;
	lat1 = abs(int(autosteer_lat)) * 100 + int(lat_min);
	lat2 = int((lat_min - int(lat_min)) * 10000000);

	lon_min = fabs(autosteer_lon - int(autosteer_lon)) * 60;
	lon1 = abs(int(autosteer_lon)) * 100 + int(lon_min);
	lon2 = int((lon_min - int(lon_min)) * 10000000);

	//GGA
	snprintf(nmea_buffer,NMEA_BUFFER_SIZE,
	         "$GNGGA,%s,%d.%d,%s,%d.%d,%s,%d,%s,%s,%.3f,M,%s,M,%s,",
	         fix_time, 
		 lat1, lat2, (autosteer_lat<0 ? "S" : "N"),
		 lon1, lon2, (autosteer_lon<0 ? "W" : "E"),
		 fix_quality,
		 num_sats,
		 hdop,
		 autosteer_altitude,
		 geoid,
		 dgps_age);

	//add checksum and CRLF
	compute_nmea_checksum(nmea_buffer, checksum);
	strncat(nmea_buffer,checksum, NMEA_BUFFER_SIZE);
	strncat(nmea_buffer,"\r\n", NMEA_BUFFER_SIZE);

	//VTG
	vtg = nmea_buffer + strnlen(nmea_buffer, NMEA_BUFFER_SIZE);
	snprintf(vtg,NMEA_BUFFER_SIZE - strnlen(nmea_buffer, NMEA_BUFFER_SIZE),
	         "$GPVTG,%.3f,T,,,%s,N,%.2f,K",
	         autosteer_heading,
		 vtg_speed_knots, //knots
		 autosteer_speed); //kph

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
	uint16_t old_rvc;
	float roll_direction;

	got_gga = false;
	got_vtg = false;

	if (imu_reverse) 
		roll_direction = -1;
	else 
		roll_direction = 1;

	//prepare to look at older IMU position
	old_rvc = (imu_lookback + current_rvc) % MAX_LOOKBACK;
	yaw = rvc_data[old_rvc].yaw;

	if (use_imu && yaw < 400) {
		//if we have a valid IMU reading, calculate the IMU
		//heading offset and do roll compensation.
		
		if (imu_use_pitch) {
			autosteer_roll = rvc_data[old_rvc].pitch * roll_direction;
		} else {
			autosteer_roll = rvc_data[old_rvc].roll * roll_direction;
		}

		autosteer_roll += imu_roll_offset;

		if(!imu_heading_offset_set && autosteer_speed <= MIN_VTG_SPEED) {
			//calculate heading based on fix to fix
			//use that to figure out IMU heading offset
			if (last_lat < 400) {
				if (haversine::distance(last_lat, last_lon,
				                        autosteer_lat,
						        autosteer_lon) > MIN_FIX_DIST) {
					autosteer_heading = haversine::bearing(
					                         last_lat, last_lon,
							         autosteer_lat, autosteer_lon);
					imu_heading_offset = autosteer_heading - yaw;
					if (imu_heading_offset < 0) imu_heading_offset += 360;
					imu_heading_offset_set = true;
				}

			} else {
				last_lat = autosteer_lat;
				last_lon = autosteer_lon;
			}
		} else if (autosteer_speed > MIN_VTG_SPEED) {
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
			autosteer_heading = fmod(yaw + imu_heading_offset,360);
			if (autosteer_heading < 0) autosteer_heading += 360;
		} else {
			//no offset yet calculated, so throw in the VTG heading
			autosteer_heading = vtg_head;
		}

		roll_rad = autosteer_roll * M_PI / 180.0;
	} else {
		//we either don't have an IMU or we haven't read it yet
		//so don't do roll compensation, but do adjust the GPS
		//as best we can to the center of the tractor from its
		//potentially offset position.
		
		roll_rad = 0;
		autosteer_heading = vtg_head;
		autosteer_roll = 0;
	}

	//do roll compensation and adjust lat and lon for the 
	//offsets of the GPS unit

	heading90 = autosteer_heading - 90;
	if (heading90 <0) heading90 +=360;

	//lateral offset from tilt of tractor
	tilt_offset = sin(roll_rad) * antenna_height;
	//lateral offset from antenna to center of tractor
	center_offset = cos(roll_rad) * antenna_right;

	//move the gps position
	haversine::move_distance_bearing(autosteer_lat,
					 autosteer_lon,
					 heading90,
					 tilt_offset + center_offset);

	//Adjust altitude
	autosteer_altitude -= cos(roll_rad) * antenna_height +
			      sin(roll_rad) * center_offset;

	if (antenna_forward) {
		haversine::move_distance_bearing(autosteer_lat,
						 autosteer_lon,
						 autosteer_heading,
						 -antenna_forward);
	}

	//calculate yaw rate.
	heading_delta = autosteer_heading - last_heading;
	if (heading_delta > 180) heading_delta -= 360; //yawing left across 0
	else if (heading_delta <= -180) heading_delta += 360; //yawing right across 0

	last_heading = autosteer_heading;
	autosteer_yawrate = heading_delta * 1000.0 / rate_timer;

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

	autosteer_datetime = ((float)seconds + hundredths /100.0) * 4;
	autosteer_datetime |= ((uint64_t)minutes << 8);
	autosteer_datetime |= ((uint64_t)hours << 16);
	//Date not part of GGA, so fake it for now. 1 Jan 1985
	autosteer_datetime |= 0x7d7d000401000000;
	
	parser.getArg(1,latitude);
	parser.getArg(2,lat_ns);

	autosteer_lat = atof(latitude);
	autosteer_lat = int(autosteer_lat / 100) /*deg part*/ +
		          (autosteer_lat - int(autosteer_lat/100) * 100) / 60; /*decimal part*/

	if (lat_ns[0] == 'S') 
		autosteer_lat = -autosteer_lat;

	parser.getArg(3,longitude);
	parser.getArg(4,lon_ew);

	autosteer_lon = atof(longitude);
	autosteer_lon = int(autosteer_lon / 100) /*deg part*/ +
		          (autosteer_lon - int(autosteer_lon/100) * 100) / 60; /*decimal part*/
	
	if (lon_ew[0] == 'W')
		autosteer_lon = -autosteer_lon;

	parser.getArg(5, fix_quality);
	switch(fix_quality) {
	case 4:
		autosteer_mode = 'R';
		break;
	case 5:
		autosteer_mode = 'F';
		break;
	case 0:
		//No GPS position at all, so ignore
		got_gga = false;
		got_vtg = false;
		return;

	default:
		autosteer_mode = 'G';
	}

	parser.getArg(8, altitude);
	autosteer_altitude = atof(altitude);

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
	autosteer_speed = atof(vtg_speed_knots);
	autosteer_speed *= 1.852; //convert to kph

	if(got_gga) {
		process_imu();
	}

}

void error_handler() {

}

int16_t get_imu_lookback(void) {
	return (MAX_LOOKBACK - imu_lookback) * 10;
}

void set_imu_lookback(int16_t new_lookback_ms) {
	if (new_lookback_ms >0 and new_lookback_ms <= (MAX_LOOKBACK * 10)) {
		imu_lookback = MAX_LOOKBACK - new_lookback_ms / 10;
	}
}

//TODO: pass in handler to call when we've got a new position and
//need to send it over CAN.
void setup_nmea_parser(FixHandler fix_handler, Stream *imu_stream = NULL, SendNMEA send_nmea_handler = NULL) {
	parser.setErrorHandler(error_handler);
	parser.addHandler("G-GGA", GGA_handler);
	parser.addHandler("G-VTG", VTG_handler);

	got_fix = fix_handler;
	send_nmea = send_nmea_handler;

	imu_heading_offset_set = false;

	//zero out imu receive buffers
	//memset(rvc_data,0,sizeof(BNO08x_RVC_Data) * MAX_LOOKBACK);
	for (int i=0; i < MAX_LOOKBACK ; i++) {
		rvc_data[i].yaw = 400; //mark slot as invalid
	}

	//initialize our timeout counters
	nmea_timer = 0;
	imu_timer = 0;
	rate_timer = 0;

	last_lat = 400;
	last_lon = 400;
	buf[2] = 0;


	if (imu_stream) {
		rvc.begin(imu_stream);
		use_imu = true;
	}
}

void read_imu() {
	if (use_imu) {
		if(rvc.read(&rvc_data[current_rvc]) ) {
			current_rvc = (current_rvc + 1) % MAX_LOOKBACK;
			imu_timer = 0;
		} else if (imu_timer > 12) {
			//if 12 ms has elapsed, IMU must have gone silent
			imu_timer = 0;
			rvc_data[current_rvc].yaw = 400; //mark invalid
			current_rvc = (current_rvc + 1) % MAX_LOOKBACK;
		}
	}
}

void nmea_process(char c) {
	read_imu();
	parser << c;
}
