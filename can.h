#ifndef __CAN_H__
#define __CAN_H__

#include "whichteensy.h"

//Use a class for multiple CAN interfaces?
namespace CAN {
	
	void setup();
	//void set_speed(int freq);
	void send_position(void);
	void send_no_position(void);

}


#endif
