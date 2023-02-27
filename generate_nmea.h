#ifndef __GENERATE_NMEA_H__
#define __GENERATE_NMEA_H__

#include <Arduino.h>

#define GNB_NMEA_BUFFER_SIZE 100

class GenerateNMEABase {
protected:
	char nmea_buffer[GNB_NMEA_BUFFER_SIZE];

public:
	bool use_corrected;
	bool nmea_valid;

public:
	GenerateNMEABase(bool imu_corrected = false) { nmea_valid = false; use_corrected = use_corrected;}
	const char *get_nmea() {
		return this->nmea_buffer;
	}
	void imu_corrected(bool corrected = false) {
		use_corrected = corrected;
	}
	virtual void generate(void) = 0;
};

#endif
