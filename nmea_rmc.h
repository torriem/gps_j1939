#ifndef __NMEA_RMC_H__
#define __NMEA_RMC_H__

#include "generate_nmea.h"

class NMEARMC : public GenerateNMEABase {
protected:
public:
	NMEARMC(bool use_imu=false) {this->use_corrected = use_imu;}
	void generate(void);
};

extern NMEARMC nmea_rmc;

#endif
