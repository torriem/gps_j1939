#include "serial_config.h"
#include "nmea_checksum.h"
#include "globals.h"
#include "whichteensy.h"
#ifndef TEENSY
#include <elapsedMillis.h>
#endif

#define SC_BUFFER_LEN 256

uint8_t serial_buffer[1024]; //overkill hopefully

static char sc_nmea_buffer[SC_BUFFER_LEN];
static char checksum[6];

/*
 * Serial NMEA outputs
 */

#if defined(ESP32)

BluetoothSerial SerialBT;
HardwareSerial SerialIMU = HardwareSerial(SERIAL_DEFAULT_IMU);
HardwareSerial SerialGPS = HardwareSerial(SERIAL_DEFAULT_GPS);
HardwareSerial SerialOut = HardwareSerial(SERIAL_DEFAULT_OUT); //if 0 it's the same as Serial

SerialNMEA bluetooth_nmea = SerialNMEA(0,"Bluetooth",&SerialBT);
SerialNMEA serialout_nmea = SerialNMEA(1,"SerialOut",&SerialOut);

#elif defined(TEENSY)
SerialNMEA serialusb_nmea(0,"SerialUSB",&Serial);
SerialNMEA serialout_nmea(1,"SerialOut",&SerialOut);
#endif

/* important NOTE */
//SerialNMEA objects must be configured in the main program with the 
//appropriate message type intervals or they won't fire.  They are
//disabled by default.

namespace serial_config {

#if defined(ESP32)
	int serialout_baud = SERIAL_DEFAULT_OUT_SPEED;
	int gps_baud = SERIAL_DEFAULT_GPS_SPEED;
	int imu_baud = SERIAL_DEFAULT_IMU_SPEED;
	char bt_name[32] = BLUETOOTH_DEFAULT_NAME; //Need a new name
#elif defined(TEENSY)
	int serialout_baud = SERIAL_DEFAULT_OUT_SPEED;
	int gps_baud = SERIAL_DEFAULT_GPS_SPEED;
	int imu_baud = SERIAL_DEFAULT_IMU_SPEED;
#endif

	static int msg_count = 0;

	void setup() {
		//TODO: read from flash eeprom last values
#if defined(ESP32)
		set_bluetooth_name(bt_name);
		set_serial_speed(SERIAL_ID_OUT, serialout_baud);
		SerialGPS.setRxBufferSize(1024);
		set_gps_speed(gps_baud);
		SerialIMU.setRxBufferSize(1024);
		set_imu_speed(imu_baud);
#elif defined(TEENSY)
		set_serial_speed(SERIAL_ID_OUT, serialout_baud);
		SerialGPS.addMemoryForRead(serial_buffer,1024);
		set_gps_speed(gps_baud);
		set_imu_speed(imu_baud);
#endif
	}

#if defined(ESP32)
	void set_bluetooth_name(char *new_bt_name) {
		strncpy(bt_name, new_bt_name,32);
		SerialBT.end();
		SerialBT.begin(new_bt_name);
	}
#endif

	void set_serial_speed(int serial_id, int baudrate) {
#if defined(ESP32)
		if (serial_id == SERIAL_ID_BT) return; //no speed to set here
		if (serial_id == SERIAL_ID_OUT) {
			SerialOut.end();
			SerialOut.begin(baudrate, SERIAL_8N1, OUT_RX, OUT_TX);
		}
#elif defined(TEENSY)
		if (serial_id == SERIAL_ID_USB) return; //speed is irrelevant here
		if (serial_id == SERIAL_ID_OUT) {
			SerialOut.end();
			SerialOut.begin(baudrate);
		}
#endif
		//save to memory
	}

	void set_gps_speed(int baudrate) {
		//Serial.print(F("Setting GPS baud to "));
		//Serial.println(baudrate);
		gps_baud = baudrate;

		SerialGPS.end();
#if defined (ESP32)
		SerialGPS.begin(baudrate, SERIAL_8N1, GPS_RX, GPS_TX);
#else
		SerialGPS.begin(baudrate);
#endif
		//save to memory
	}

	void set_imu_speed(int baudrate) {
		imu_baud = baudrate;

		SerialIMU.end();
#if defined (ESP32)
		SerialIMU.begin(baudrate, SERIAL_8N1, IMU_RX, IMU_TX);
#else
		SerialIMU.begin(baudrate);
#endif
		//save to memory
	}

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
		         "$PTSER,%d,%s,%s,%d,%d,%d,%d",
			 0, F("Bluetooth"),
			 bt_name,
			 bluetooth_nmea.get_gga_interval(),
			 bluetooth_nmea.get_vtg_interval(),
			 serialout_nmea.get_rmc_interval(),
			 bluetooth_nmea.get_cfg_interval());

		//add checksum and CRLF
		compute_nmea_checksum(sc_nmea_buffer, checksum);
		strncat(sc_nmea_buffer,checksum, SC_BUFFER_LEN);
		strncat(sc_nmea_buffer,"\r\n", SC_BUFFER_LEN);
		msg_len = strlen(sc_nmea_buffer);
		msg = sc_nmea_buffer + msg_len;

		snprintf(msg, SC_BUFFER_LEN - msg_len,
		         "$PTSER,%d,%s,%d,%d,%d,%d,%d",
			 1, F("Serial1"),
			 serialout_baud,
			 serialout_nmea.get_gga_interval(),
			 serialout_nmea.get_vtg_interval(),
			 serialout_nmea.get_rmc_interval(),
			 serialout_nmea.get_cfg_interval());
		//add checksum and CRLF
		compute_nmea_checksum(msg, checksum);
		strncat(msg, checksum, SC_BUFFER_LEN - msg_len);
		strncat(sc_nmea_buffer,"\r\n", SC_BUFFER_LEN - msg_len);

#elif defined (TEENSY)
		snprintf(sc_nmea_buffer, SC_BUFFER_LEN,
		         "$PTSER,%d,%s,%d,%d,%d,%d,%d",
			 0, "SerialUSB",
			 115200,
			 serialusb_nmea.get_gga_interval(),
			 serialusb_nmea.get_vtg_interval(),
			 serialout_nmea.get_rmc_interval(),
			 serialusb_nmea.get_cfg_interval());

		//add checksum and CRLF
		compute_nmea_checksum(sc_nmea_buffer, checksum);

		strncat(sc_nmea_buffer,checksum, SC_BUFFER_LEN);
		strncat(sc_nmea_buffer,"\r\n", SC_BUFFER_LEN);

		msg_len = strlen(sc_nmea_buffer);
		msg = sc_nmea_buffer + msg_len;

		snprintf(msg, SC_BUFFER_LEN - msg_len,
		         "$PTSER,%d,%s,%d,%d,%d,%d,%d",
			 1, F("SerialOut"),
			 serialout_baud,
	 		 serialout_nmea.get_gga_interval(),
			 serialout_nmea.get_vtg_interval(),
			 serialout_nmea.get_rmc_interval(),
			 serialout_nmea.get_cfg_interval());
		//add checksum and CRLF
		compute_nmea_checksum(msg, checksum);
		strncat(msg, checksum, SC_BUFFER_LEN - msg_len);
		strncat(sc_nmea_buffer,"\r\n", SC_BUFFER_LEN - msg_len);
#endif
	}

	const char *get_nmea() {
		return sc_nmea_buffer;
	}

}
