#include "nmea_vtg.h"
#include "nmea_checksum.h"
#include <math.h>
#include "globals.h"

NMEAVTG nmea_vtg();

void NMEAVTG::generate(void) {
	char checksum[6];

	snprintf(this->nmea_buffer,GNB_NMEA_BUFFER_SIZE,
	         "$GNVTG,%.3f,T,,M,%.3f,N,%.2f,K,D",
	         gps_heading,
		 gps_speed * 0.54, //knots
		 gps_speed); //kph

	//add checksum and CRLF
	compute_nmea_checksum(this->nmea_buffer, checksum);
	strncat(this->nmea_buffer,checksum, GNB_NMEA_BUFFER_SIZE);
	strncat(this->nmea_buffer,"\r\n", GNB_NMEA_BUFFER_SIZE);
}
