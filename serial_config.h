#ifndef __SERIAL_CONFIG_H__
#define __SERIAL_CONFIG_H__

#include "whichteensy.h"
#include "defaults.h"
#include "serial_nmea.h"

#ifndef NULL
#define NULL 0
#endif

#if defined(ESP32)
#include <BluetoothSerial.h>
extern SerialNMEA bluetooth_nmea;
extern SerialNMEA serialout_nmea;
extern BluetoothSerial SerialBT;
extern HardwareSerial SerialIMU;
extern HardwareSerial SerialGPS;

#elif defined(TEENSY)
extern SerialNMEA serialusb_nmea;
extern SerialNMEA serialout_nmea;
extern uint8_t serialgps_buffer[1024];
#define SerialIMU SERIAL_DEFAULT_IMU
#define SerialGPS SERIAL_DEFAULT_GPS
#define SerialOut SERIAL_DEFAULT_OUT

#endif

namespace serial_config {

	extern int serialout_baud;
	extern int gps_baud;
	extern int imu_baud;

#if defined(ESP32)
	extern char bt_name[32];

	void set_bluetooth_name(char *bluetooth_name);
#endif
	void setup();
	void set_serial_speed(int serial_id, int baudrate);
	void set_gps_speed(int baudrate);
	void set_imu_speed(int baudrate);

	void read_flash();
	void write_flash();
	

	void generate_nmea(void);
	const char *get_nmea();
}

#endif
