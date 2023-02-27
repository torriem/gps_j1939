#ifndef __NMEA_GGA_H__
#define __NMEA_GGA_H__


#include "generate_nmea.h"

class NMEAGGA : public GenerateNMEABase {
protected:
public:
	NMEAGGA(bool use_imu=false) {this->use_corrected = use_imu;}
	void generate(void);
};

#endif
