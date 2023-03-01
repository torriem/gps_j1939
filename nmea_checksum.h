#ifndef __NMEA_CHECKSUM_H__
#define __NMEA_CHECKSUM_H__

#include <Arduino.h>

static inline void compute_nmea_checksum(const char *nmea, char *checksum) {
	int sum = 0;

	for (unsigned int i=1 ; i< strlen(nmea) ; i++) {
		sum ^= nmea[i];
	}

	snprintf(checksum,6,"*%02X", sum);
}

#endif


