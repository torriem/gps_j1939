#include "serial_config.h"
#include "nmea_checksum.h"
#include "globals.h"
#include "whichteensy.h"
#ifndef TEENSY
#include <elapsedMillis.h>
#endif

#define SC_BUFFER_LEN 256

static char sc_nmea_buffer[SC_BUFFER_LEN];
static char checksum[6];

/*
 * Serial NMEA outputs
 */

#if defined(ESP32)
//These must be configured in the main program with the appropriate
//Serial device instances.
SerialNMEA bluetooth_nmea = SerialNMEA(0,"Bluetooth",NULL);
SerialNMEA serialout_nmea = SerialNMEA(1,"SerialOut", NULL);
//SerialNMEA serial2_nmea = SerialNMEA(2,"Serial2", NULL);
//SerialNMEA serial3_nmea = SerialNMEA(3,"Serial3", NULL);
#elif defined(TEENSY)
SerialNMEA serialusb_nmea(0,"SerialUSB",NULL);
SerialNMEA serialout_nmea(1,"SerialOut",NULL);
#endif

namespace serial_config {

#if defined(ESP32)
	int serialout_baud = 115200;
	int gps_baud = 460800;
	int imu_baud = 115200;
	char bt_name[32] = "gps_j1939"; //Need a new name
#elif defined(TEENSY)
	int serialout_baud = 115200;
	int gps_baud = 115200;
	int imu_baud = 115200;
#endif

	static int msg_count = 0;


	void read_flash() {
		//TODO
	}

	void write_flash() {
		//TODO
	}

	void generate_nmea(void) {
		char *msg;
		int msg_len;
	 
#if defined(ESP32)	
		snprintf(sc_nmea_buffer, SC_BUFFER_LEN,
		         "$PTSER,%d,%s,%s,%s,%s,%s",
			 0, F("Bluetooth"),
			 bt_name,
			 bluetooth_nmea.get_gga(),
			 bluetooth_nmea.get_vtg(),
			 bluetooth_nmea.get_cfg());

		//add checksum and CRLF
		compute_nmea_checksum(sc_nmea_buffer, checksum);
		strncat(sc_nmea_buffer,checksum, SC_BUFFER_LEN);
		strncat(sc_nmea_buffer,"\r\n", SC_BUFFER_LEN);
		msg_len = strlen(sc_nmea_buffer);
		msg = sc_nmea_buffer + msg_len;

		snprintf(msg, SC_BUFFER_LEN - msg_len,
		         "$PTSER,%d,%s,%d,%s,%s,%s",
			 1, F("Serial1"),
			 serialout_baud,
			 serialout_nmea.get_gga(),
			 serialout_nmea.get_vtg(),
			 serialout_nmea.get_cfg());
		//add checksum and CRLF
		compute_nmea_checksum(msg, checksum);
		strncat(msg, checksum, SC_BUFFER_LEN - msg_len);
		strncat(sc_nmea_buffer,"\r\n", SC_BUFFER_LEN - msg_len);

#elif defined (TEENSY)
		snprintf(sc_nmea_buffer, SC_BUFFER_LEN,
		         "$PTSER,%d,%s,%d,%s,%s,%s",
			 0, "SerialUSB",
			 115200,
			 serialusb_nmea.get_gga(),
			 serialusb_nmea.get_vtg(),
			 serialusb_nmea.get_cfg());

		//add checksum and CRLF
		compute_nmea_checksum(sc_nmea_buffer, checksum);

		strncat(sc_nmea_buffer,checksum, SC_BUFFER_LEN);
		strncat(sc_nmea_buffer,"\r\n", SC_BUFFER_LEN);

		msg_len = strlen(sc_nmea_buffer);
		msg = sc_nmea_buffer + msg_len;

		snprintf(msg, SC_BUFFER_LEN - msg_len,
		         "$PTSER,%d,%s,%d,%s,%s,%s",
			 1, F("SerialOut"),
			 serialout_baud,
			 "gga",
			 "vtg",
			 "cfg"
			 /*serialout_nmea.get_gga(),
			 serialout_nmea.get_vtg(),
			 serialout_nmea.get_cfg()*/);
		//add checksum and CRLF
		compute_nmea_checksum(msg, checksum);
		strncat(msg, checksum, SC_BUFFER_LEN - msg_len);
		strncat(sc_nmea_buffer,"\r\n", SC_BUFFER_LEN - msg_len);
#endif
	}
		
	void send_nmea(scfg_process_imu process_imu) {
#if defined(ESP32)
		if (bluetooth_nmea.is_active_cfg()) {
			bluetooth_nmea.send_string(sc_nmea_buffer, strlen(sc_nmea_buffer));
		}
#endif
		//because serial writing blocks sometimes, we should process
		//the IMU since it comes in at 100hz
		if(process_imu) process_imu();

		if (serialout_nmea.is_active_cfg()) {
			serialout_nmea.send_string(sc_nmea_buffer, strlen(sc_nmea_buffer));
		}
		//because serial writing blocks sometimes, we should process
		//the IMU since it comes in at 100hz
		if(process_imu) process_imu();

#if defined(TEENSY)
		if (serialusb_nmea.is_active_cfg()) {
			serialusb_nmea.send_string(sc_nmea_buffer, strlen(sc_nmea_buffer));
		}
#endif

	}

	const char *get_nmea() {
		return sc_nmea_buffer;
	}

}
