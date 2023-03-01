#ifndef __SERIAL_CONFIG_H__
#define __SERIAL_CONFIG_H__


namespace serial_config {

#if defined(ESP32)
	extern int serial1_baud;
	extern int gps_baud;
	extern int imu_baud;
	extern char bt_name[32];
#elif defined(TEENSY)

#endif

	void read_flash();
	void write_flash();
	

	void generate_nmea(void);

	void send_nmea(void);

	const char *get_nmea();
}

#endif
