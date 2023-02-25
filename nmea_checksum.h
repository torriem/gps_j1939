#ifndef __NMEA_CHECKSUM_H__
#define __NMEA_CHECKSUM_H__

static inline void compute_nmea_checksum(const char *nmea, char *checksum) {
	int sum = 0;

	for (unsigned int i=0 ; i< strlen(nmea) ; i++) {
		sum ^= nmea[i];
	}

	snprintf(checksum,6,"*%02X", sum);
}

#endif


