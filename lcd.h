#ifndef __LCD_H__
#define __LCD_H__

#include "whichteensy.h"
//Turn on LCD on Teensy by setting this define, disable
//by commenting it out.
//#define TEENSY_TFT 

namespace LCD {
	void setup();
	void heartbeat(); //show spinning indicator
	void new_position();
}

#endif
