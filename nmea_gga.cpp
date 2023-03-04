#include "nmea_gga.h"
#include "nmea_checksum.h"
#include <math.h>
#include "globals.h"

NMEAGGA nmea_gga = NMEAGGA(true);
NMEAGGA nmea_gga_uncorrected = NMEAGGA(false);

void NMEAGGA::generate(void) {
	char checksum[6];
	char dgps_age[6];

	double lat, lon, lat_min, lon_min;
	int lat1, lat2, lon1, lon2;

	snprintf(dgps_age,6,"%.1f", gps_dgps_age);

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

	//GGA
	snprintf(this->nmea_buffer,GNB_NMEA_BUFFER_SIZE,
	         "$GPGGA,%s,%d.%d,%s,%d.%d,%s,%d,%02d,%.2f,%.2f,M,%.1f,M,%s,0000",
	         gps_fix_time, 
		 lat1, lat2, (lat<0 ? "S" : "N"),
		 lon1, lon2, (lon<0 ? "W" : "E"),
		 gps_mode,
		 gps_num_sats,
		 gps_hdop,
		 gps_altitude,
		 gps_geoid,
		 (gps_dgps_age > 0 ? dgps_age : ""));

	//add checksum and CRLF
	compute_nmea_checksum(this->nmea_buffer, checksum);
	strncat(this->nmea_buffer,checksum, GNB_NMEA_BUFFER_SIZE);
	strncat(this->nmea_buffer,"\r\n", GNB_NMEA_BUFFER_SIZE);
}
