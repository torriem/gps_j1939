#ifndef __PX1172RH_H__
#define __PX1172RH_H__

#include "fix_handler.h"

namespace px1172rh {
	void setup(FixHandler fix_handler);
	void set_on_fix_handler(FixHandler fix_handler);
	void process_byte(char c);
}

#endif


