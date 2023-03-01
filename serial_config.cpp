#include "serial_config.h"
#include "nmea_checksum.h"
#include "globals.h"
#ifndef TEENSY
#include <elapsedMillis.h>
#endif

#define SC_BUFFER_LEN 256

static char nmea_buffer[SC_BUFFER_LEN];
static char checksum[6];

namespace serial_config {
	int serial1_baud = 115200;
	int gps_baud = 460800;
	int imu_baud = 115200;
	char bt_name[32] = "gps_j1939"; //Need a new name
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
		snprintf(nmea_buffer, SC_BUFFER_LEN,
		         "$PTSER,%d,%s,%s,%s,%s,%s",
			 0, F("Bluetooth"),
			 bt_name,
			 bluetooth_nmea.get_gga(),
			 bluetooth_nmea.get_vtg(),
			 bluetooth_nmea.get_cfg());

		//add checksum and CRLF
		compute_nmea_checksum(nmea_buffer, checksum);
		strncat(nmea_buffer,checksum, SC_BUFFER_LEN);
		strncat(nmea_buffer,"\r\n", SC_BUFFER_LEN);
		msg_len = strlen(nmea_buffer);
		msg = nmea_buffer + msg_len;

		snprintf(msg, SC_BUFFER_LEN - msg_len,
		         "$PTSER,%d,%s,%d,%s,%s,%s",
			 1, F("Serial1"),
			 serial1_baud,
			 serial1_nmea.get_gga(),
			 serial1_nmea.get_vtg(),
			 serial1_nmea.get_cfg());
		//add checksum and CRLF
		compute_nmea_checksum(msg, checksum);
		strncat(msg, checksum, SC_BUFFER_LEN - msg_len);
		strncat(nmea_buffer,"\r\n", SC_BUFFER_LEN - msg_len);

#elif defined (TEENSY)

#endif
	}
		
	void send_nmea(void) {
		if (bluetooth_nmea.is_active_cfg()) {
			bluetooth_nmea.send_string(nmea_buffer, strlen(nmea_buffer));
		}
		if (serial1_nmea.is_active_cfg()) {
			serial1_nmea.send_string(nmea_buffer, strlen(nmea_buffer));
		}
	}

	const char *get_nmea() {
		return nmea_buffer;
	}

}
