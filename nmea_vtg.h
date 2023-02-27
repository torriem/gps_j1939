#ifndef __NMEA_VTG_H__
#define __NMEA_VTG_H__


#include "generate_nmea.h"

class NMEAVTG : public GenerateNMEABase {
protected:
public:
	NMEAVTG(bool use_imu=false) {use_corrected = use_imu;}
	void generate(void);
};

#endif
