#include "nmea_rmc.h"
#include "nmea_checksum.h"
#include <math.h>
#include "globals.h"

NMEARMC nmea_rmc = NMEARMC(true);
NMEARMC nmea_rmc_uncorrected = NMEARMC(false);

void NMEARMC::generate(void) {
	char checksum[6];
	char mode_indicator;

	double lat, lon, lat_min, lon_min;
	int lat1, lat2, lon1, lon2;

	if (use_corrected) {
		lat = gps_latitude;
		lon = gps_longitude;
	} else {
		lat = gps_orig_latitude;
		lon = gps_orig_longitude;
	}

	lat_min = fabs(lat - int(lat)) * 60;
	lat1 = abs(int(lat)) * 100 + int(lat_min);
	lat2 = int((lat_min - int(lat_min)) * 10000000);

	lon_min = fabs(lon - int(lon)) * 60;
	lon1 = abs(int(lon)) * 100 + int(lon_min);
	lon2 = int((lon_min - int(lon_min)) * 10000000);

	switch (gps_mode) {
	case 5:
		mode_indicator = 'F';
		break;
	case 4:
		mode_indicator = 'R';
		break;
	case 2:
		mode_indicator = 'D';
		break;
	case 1:
		mode_indicator = 'A';
		break;
	default:
		mode_indicator = 'N';
		break;
	}

	//GGA
	snprintf(this->nmea_buffer,GNB_NMEA_BUFFER_SIZE,
	         "$GPRMC,%s,A,%d.%d,%s,%d.%d,%s,%.2f,%.2f,%s,%c,V",
	         gps_fix_time, 
		 lat1, lat2, (lat<0 ? "S" : "N"),
		 lon1, lon2, (lon<0 ? "W" : "E"),
		 gps_speed * 0.53996, //knots
		 gps_heading,
		 gps_fix_date,
		 mode_indicator);
		 

	//add checksum and CRLF
	compute_nmea_checksum(this->nmea_buffer, checksum);
	strncat(this->nmea_buffer,checksum, GNB_NMEA_BUFFER_SIZE);
	strncat(this->nmea_buffer,"\r\n", GNB_NMEA_BUFFER_SIZE);
}
