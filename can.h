#ifndef __CAN_H__
#define __CAN_H__

#include "whichteensy.h"

//Use a class for multiple CAN interfaces?
namespace CAN {
	
#if defined(TEENSY)  //TODO: or defined(ESP32) or maybe even the Due

	void setup();
	//void set_speed(int freq);
	void send_position(void);
	void send_no_position(void);

#endif

}


#endif
