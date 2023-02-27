#include "px1172rh.h"
#include "globals.h"
#include "shared_nmea_buffer.h"
#include "stinmea.h"
#include "haversine.h"
#include "kfilter1.h"
#include "SimpleKalmanFilter.h"
#include "math.h"
	
static uint16_t year=0; 
static uint8_t month=0, day=0;
static uint8_t hour=0, minute=0, seconds=0, hundredths=0;
static double latitude=0;
static double longitude=0;
static double altitude=0;
static double heading=0;
static double heading90=0;
static double roll=0;
static uint8_t fix30_mode;
static bool got_pos=false;
static bool got_attitude=false;

static float last_heading = 0;
static uint32_t last_heading_time = 0;

static char sti_nmea_buffer[160];
static STINMEA sti(sti_nmea_buffer, sizeof(sti_nmea_buffer)); //handles PSTI

static KFilter1 yawrate_filter(0.1, 1.0f, 0.0003f);
static KFilter1 heading_filter(0.1, 1.0f, 0.0003f);
static KFilter1 roll_filter(0.1, 1.0f, 0.0003f);

static FixHandler got_psti_fix = NULL;
static SendNMEA send_nmea = NULL;

static inline void generate_nmea(void) {
	/* generate new NMEA and VTG sentences based on
	 * our corrected position and heading
	 */

	#if 0
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
	#endif
}


void setup_psti(FixHandler fix_handler){
	got_psti_fix = fix_handler;
}

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

				heading = atan2(-east_speed, north_speed) * 180.0 / PI;
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
