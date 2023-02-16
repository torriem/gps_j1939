#include "nmeaimu.h"
#include "Adafruit_BNO08x_RVC.h"
#include "NMEAParser.h"
#include "haversine.h"
#include "kfilter1.h"
#include "SimpleKalmanFilter.h"
#include "math.h"
	
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

static KFilter1 yawrate_filter(0.1, 1.0f, 0.0003f);
static KFilter1 heading_filter(0.1, 1.0f, 0.0003f);
static KFilter1 roll_filter(0.1, 1.0f, 0.0003f);

static FixHandler got_fix;

static NMEAParser<2> parser;

//Use adafruit's library or AOG's modified library?
static Adafruit_BNO08x_RVC rvc;
static BNO08x_RVC_Data rvc_data;
static bool  use_imu = false;

static bool got_gga = false;
static bool got_vtg = false;

void GGA_handler() {

}

void VTG_handler() {

}

void error_handler() {

}

//TODO: pass in handler to call when we've got a new position and
//need to send it over CAN.
void setup_nmea_parser(FixHandler fix_handler, Stream *imu_stream = NULL) {
	parser.setErrorHandler(error_handler);
	parser.addHandler("G-GGA", GGA_handler);
	parser.addHandler("G-VTG", VTG_handler);

	got_fix = fix_handler;

	if (imu_stream) {
		rvc.begin(imu_stream);
		use_imu = true;
	}
}

inline void read_imu() {
	if (use_imu) {
		rvc.read(&rvc_data);
	}
}

void nmea_process(char c) {
	read_imu();
	parser << c;
}
