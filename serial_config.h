#ifndef __SERIAL_CONFIG_H__
#define __SERIAL_CONFIG_H__

#include "whichteensy.h"
#include "serial_nmea.h"

#ifndef NULL
#define NULL 0
#endif

#if defined(ESP32)
extern SerialNMEA bluetooth_nmea;
extern SerialNMEA serialout_nmea;
#elif defined(TEENSY)
extern SerialNMEA serialusb_nmea;
extern SerialNMEA serialout_nmea;
#endif

namespace serial_config {

extern int serialout_baud;
extern int gps_baud;
extern int imu_baud;

#if defined(ESP32)
	extern char bt_name[32];

#endif

	void read_flash();
	void write_flash();
	

	void generate_nmea(void);
	const char *get_nmea();
}

#endif
