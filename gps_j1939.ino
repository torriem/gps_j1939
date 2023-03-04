/*
    This is a fork of my gps_j1939 project, but without any CAN
    support for ESP32. It's purpose is to take in GPS data from
    NMEA (single GPS with IMU or dual GPS), correct it for roll,
    and output new GGA and VTG NMEA messages over a serial port.
    
    This could be fed into AgOpenGPS directly (without need for
    IMU support in AOG), or into an external device through an
    RS232 transceiver.

    /Copyright 2018-2022 Michael Torrie
    torriem@gmail.com
 */

#include <math.h>
#include "globals.h"
#include "circle_generator.h"
#include "static_position.h"
#include "px1172rh.h"
#include "nmeaimu.h"
#include "shared_nmea_buffer.h"
#include "nmea_checksum.h"
#include "whichteensy.h"
#ifndef TEENSY
#include <BluetoothSerial.h> //ESP32 only
#include <elapsedMillis.h>
#endif
#include "can.h"
#include "lcd.h"
#include "bnorvc.h"
#include "serial_config.h"
#include "serial_csv.h"
#include "nmea_gga.h"
#include "nmea_vtg.h"

uint8_t serial_buffer[1024]; //overkill hopefully

#define RADIANS(deg) deg * M_PI / 180.0
#define DEGREES(rad) rad * 180.0 / M_PI
#define INCHES 0.0254
#define FEET 0.3048

//types of GPS
#define GPS_PX1172RH 1 //dual gps from px1172rh
//#define GPS_DUAL_F9P //FUTURE: dual F9P support
#define GPS_NMEA_BNO 3  //single GPS GGA+VTG and the BNO08x

//types of virtual position generators
#define VIRTUAL_NONE 0 //no virtual position generation
#define VIRTUAL_CIRCLE 10 //simulate moving in a perfect circle
#define VIRTUAL_STATIC 11 //arbitrary static position.

#define GPS_TIMEOUT 400 //after 200ms of no GPS position, show "No GPS" on monitor

uint8_t gps_source = GPS_PX1172RH;
uint8_t virtual_source = VIRTUAL_NONE;

//int8_t monitor_can = -1;
int8_t monitor_can = 0;

int autosteer_source=0; //0 = inadequate, 1=WAAS, 2 = SF1 or higher, 3 = External
long autosteer_lastext=0;

unsigned long last_61184 = millis();

long can_messages_received = 0; // a sort of heartbeat on the CAN bus.
uint8_t spinner_state = 0;

elapsedMillis last_good_fix;

//TODO move these to serial_config.c
#if defined(ESP32)

    BluetoothSerial SerialBT;
    HardwareSerial SerialIMU(1);
    HardwareSerial SerialGPS(2);

#elif defined(TEENSY)

     uint8_t serialgps_buffer[1024];
#    define SerialIMU Serial4
#    define SerialGPS Serial3
#    define SerialOut Serial5

#endif

SerialCSV debug_csv;
BNORVC bno_rvc;

static int message_count = 0;

void on_new_position(void) {
	elapsedMillis timer = 0;
	last_good_fix = 0;

	nmea_gga.generate();
	nmea_gga_uncorrected.generate();
	nmea_vtg.generate();

	//send to CAN bus
#if defined(ESP32)	
	bluetooth_nmea.send_position();
#endif

#if defined(TEENSY)
	LCD::new_position();
	CAN::send_position();
	serialusb_nmea.send_position();
#endif
	serialout_nmea.send_position();
	debug_csv.send_position(bno_rvc);

	message_count ++;
	if (message_count == 5) {
		//every 5 message sets, send the configuration
		//messages.
		message_count = 0;
		serial_config::send_nmea(process_imu);

		//send imu config
		//send gps config
	}
}

void on_no_position(void) {
	//It's been 400 ms since last fix, so assume GPS is lost
	CAN::send_no_position();
}

void process_imu() {
	bno_rvc.process_data();
}

void setup()
{

	//TODO: store serial input baud rates in flash
	//allow them to be changed.  And move this code to a
	//setup() in a serial module
#if defined(ESP32)
	Serial.begin(serial_config::serialout_baud);
	SerialGPS.begin(serial_config::gps_baud,SERIAL_8N1,25,14);
	SerialGPS.setRxBufferSize(1024);
	SerialIMU.begin(serial_config::imu_baud,SERIAL_8N1,27,16);
	SerialIMU.setRxBufferSize(1024);

	//TODO: store bt name and output serial baud rates
	//in flash, and allow them to be changed.
	SerialBT.begin("rovertest");

	//set up output streams
	bluetooth_nmea.set_stream(&SerialBT);
	serialout_nmea.set_stream(&Serial);

	bluetooth_nmea.send_gga(true);
	bluetooth_nmea.send_vtg(true);
	bluetooth_nmea.set_corrected(true);
#elif defined(TEENSY)
	Serial.begin(115200);
	SerialGPS.addMemoryForRead(serial_buffer,1024);
	SerialGPS.begin(serial_config::gps_baud);
	SerialIMU.begin(serial_config::imu_baud);
	SerialOut.begin(serial_config::serialout_baud);

	CAN::setup(); //set up the CAN interface
	LCD::setup(); //set up LCD if enabled

	//set up output streams
	serialusb_nmea.set_stream(&Serial);
	//serialout_nmea.set_stream(&SerialOut);

	serialusb_nmea.send_gga(true);
	serialusb_nmea.send_vtg(true);
	serialusb_nmea.send_cfg(true);
	serialusb_nmea.set_corrected(true);
#endif

	//set BNO to read data from SerialIMU
	bno_rvc.set_uart(&SerialIMU);

	debug_csv.set_stream(&Serial);

	serialout_nmea.send_gga(true);
	serialout_nmea.send_vtg(true);
	serialout_nmea.send_cfg(true);
	serialout_nmea.set_corrected(true);



	//prepare the proprietary messages that
	//contain our present settings, ultimately to help
	//configure and control over a serial port, perhaps
	//through another ESP32 if this was running on a teensy.
	delay(2000);
	Serial.println("configuring config sentences.");
	serial_config::generate_nmea();
	Serial.println("gps_j1939 on ESP32");
	autosteer_source = 0;
	gps_latitude = 0;
	gps_longitude = 0;

	switch(gps_source) {
	case GPS_PX1172RH:
		px1172rh::setup(on_new_position);
		break;
	case GPS_NMEA_BNO:
		//TODO: IMU serial port
		//setup_nmea_parser(send_gps_messages, (Stream *) NULL);
		nmea_imu::setup(on_new_position, &bno_rvc);
		break;
	}
}

void loop()
{
	char c;
	
	unsigned long t;

	// virtual position generators
	//TODO: fix virtual sources to have a callback 
	CircleGenerator circlegen;
        StaticPosition staticpos;

	//example of setup
	//circlegen.set_circle(latitude, longitude, diameter, altitude);
        //circlegen.start(millis(), 120, 22); //22 kph for 120 seconds
	//virtual_source = VIRTUAL_CIRCLE;

	//staticpos.set_position(latitude, longitude, altitude, heading);
        //staticpos.start(millis(), 120); //static position for 2 minutes
	//virtual_source = VIRTUAL_STATIC;

	while(1) {
		if (last_good_fix > GPS_TIMEOUT) {
			on_no_position();
		}
		
		LCD::heartbeat();

		t = millis();

		switch(virtual_source) {
		//TODO: fix virtual sources to have a callback 
		case VIRTUAL_CIRCLE:
			if (circlegen.circle_position(t, 200)) {
				autosteer_source = 3;
				//send_gps_messages();
				autosteer_lastext = t;
			}
			if (circlegen.finished(t))
				virtual_source = VIRTUAL_NONE;
			break;
		case VIRTUAL_STATIC:
			if (staticpos.static_position(t, 200)) {
				autosteer_source = 3;
				//send_gps_messages();
				autosteer_lastext = t;
			}
			if (staticpos.finished(t))
				virtual_source = VIRTUAL_NONE;
			break;
		}

#if defined(ESP32)	
		//process bluetooth ntrip
		while (SerialBT.available())
		{
			SerialGPS.write(SerialBT.read());
		}
#endif

		//process IMU data
		//bno_rvc.process_data();

		//now process serial bytes that have accumulated
		while(SerialGPS.available()) {
			c = SerialGPS.read();

			if (virtual_source) {
				/* ignore real GPS while virtual positions are
				   being used. */
				continue;
			}

			switch(gps_source) {
			case GPS_PX1172RH:
				px1172rh::process_byte(c);
				break;
			case GPS_NMEA_BNO:
				nmea_imu::process_byte(c);
				break;
			}
		}
	}
}
