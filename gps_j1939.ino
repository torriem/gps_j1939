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
#include "defaults.h"
#include "globals.h"
#include "circle_generator.h"
#include "static_position.h"
#include "px1172rh.h"
#include "nmeaimu.h"
#include "shared_nmea_buffer.h"
#include "nmea_checksum.h"
#include "whichteensy.h"
#ifndef TEENSY
#include <elapsedMillis.h>
#endif
#include "can.h"
#include "lcd.h"
#include "bnorvc.h"
#include "serial_config.h"
#include "serial_csv.h"
#include "nmea_gga.h"
#include "nmea_vtg.h"
#include "nmea_rmc.h"


uint8_t gps_source = GPS_DEFAULT_SOURCE;
uint8_t virtual_source = GPS_DEFAULT_VIRTUAL_SOURCE;

elapsedMillis last_good_fix;

SerialCSV debug_csv;
BNORVC bno_rvc;

void on_new_position(void) {
	elapsedMillis timer = 0;
	last_good_fix = 0;

	//TODO move these generate calls somewhere
	nmea_gga.generate();
	nmea_gga_uncorrected.generate();
	nmea_rmc.generate();
	nmea_rmc_uncorrected.generate();
	nmea_vtg.generate();

	//send to CAN bus
#if defined(ESP32)	
	bluetooth_nmea.send_position(process_imu);
#endif

#if defined(TEENSY)
	LCD::new_position();
	CAN::send_position();
	serialusb_nmea.send_position(process_imu);
#endif
	serialout_nmea.send_position(process_imu);
	debug_csv.send_position(bno_rvc);
	//Serial.println(timer);
	//Serial.println(last_good_fix);
}

void on_no_position(void) {
	//It's been 400 ms since last fix, so assume GPS is lost
	CAN::send_no_position();
	last_good_fix = 0;
}

void process_imu() {
	bno_rvc.process_data();
}

void setup()
{
	Serial.begin(115200);
	delay(2000);

	//set up ports
	serial_config::setup();



#if defined(ESP32)
	Serial.println("gps_j1939 on ESP32");
	//set up output streams
	bluetooth_nmea.set_stream(&SerialBT);
	serialout_nmea.set_stream(&Serial);

	bluetooth_nmea.set_gga_interval(5);
	bluetooth_nmea.set_vtg_interval(5);
	bluetooth_nmea.set_corrected(true);
#elif defined(TEENSY)
	Serial.println("gps_j1939 on Teensy");

	CAN::setup(); //set up the CAN interface
	LCD::setup(); //set up LCD if enabled

	//set up output streams
	serialusb_nmea.set_stream(&Serial);
	serialout_nmea.set_stream(&SerialOut);

	//serialusb_nmea.set_gga_interval(1);
	//serialusb_nmea.set_vtg_interval(1);
	//serialusb_nmea.set_rmc_interval(1);
	//serialusb_nmea.set_cfg_interval(5);
	//serialusb_nmea.set_corrected(true);

#endif
	//set BNO to read data from SerialIMU
	bno_rvc.set_uart(&SerialIMU);

	//debug_csv.set_stream(&Serial);

	serialout_nmea.set_gga_interval(1);
	serialout_nmea.set_vtg_interval(1);
	serialout_nmea.set_rmc_interval(1);
	serialout_nmea.set_cfg_interval(5);
	serialout_nmea.set_corrected(true);



	//prepare the proprietary messages that
	//contain our present settings, ultimately to help
	//configure and control over a serial port, perhaps
	//through another ESP32 if this was running on a teensy.
	Serial.println("configuring config sentences.");
	serial_config::generate_nmea();
	gps_latitude = 0;
	gps_longitude = 0;

	switch(gps_source) {
	case GPS_PX1172RH:
		Serial.println("Configured for PX1172RH Dual GPS.");
		px1172rh::setup(on_new_position);
		break;
	case GPS_NMEA_BNO:
		Serial.println("Configured for NMEA GGA and VTG, and BNO.");
		//TODO: IMU serial port
		nmea_imu::setup(on_new_position, &bno_rvc);
		break;
	}
}

void loop()
{
	char c;

	elapsedMillis mt;
	elapsedMillis blank;
	int mtype_i = 0;
	
	mt = 0;

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

		/*
		switch(virtual_source) {
		//TODO: fix virtual sources to have a callback 
		case VIRTUAL_CIRCLE:
			if (circlegen.circle_position(t, 200)) {
				//autosteer_source = 3;
				on_new_position();
				//autosteer_lastext = t;
			}
			if (circlegen.finished(t))
				virtual_source = VIRTUAL_NONE;
			break;
		case VIRTUAL_STATIC:
			if (staticpos.static_position(t, 200)) {
				//autosteer_source = 3;
				on_new_position();
				//autosteer_lastext = t;
			}
			if (staticpos.finished(t))
				virtual_source = VIRTUAL_NONE;
			break;
		}
		*/

		//process IMU data
		//bno_rvc.process_data();

		/*
		if (mt > 80) {
			//more than 80 ms elapsed since the beginning of the
			//NMEA message cluster.  We must be between frames
			in_set = false;
		}
		*/

		//now process serial bytes that have accumulated
		while(SerialGPS.available()) {
			c = SerialGPS.read();

			//watch for the $G-GGA NMEA message for timing purposes
			if (c == '$') {
				mtype_i = 0;
			} else if (mtype_i < 5) {
				//Look for GGA message type for timing.  GGA nearly 
				//always is near the first of the messages sent in
				//each bundle.
				switch(mtype_i) {
				case 0:
				case 2:
				case 3:
					//If we aren't matching G_GGA, then this
					//is not the message we're timing off of.
					if (c != 'G')
						mtype_i = 5;
					break;
				case 4:
					if (c == 'A') {
						//Start our timing from this point
						//so fetch the IMU reading we're going
						//to use for this set of messages.
						//TODO: make this go through base case for generic code
						process_imu();
						imu_current_roll = bno_rvc.get_roll_ave(imu_lookback, imu_window);
						imu_current_pitch = bno_rvc.get_pitch(imu_lookback);
						imu_current_yaw = bno_rvc.get_pitch(imu_lookback);
						Serial.println(mt);
						mt = 0;
					}
				}
				mtype_i ++;
			}

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

#if defined(ESP32)	
		//process bluetooth ntrip
		while (SerialBT.available())
		{
			SerialGPS.write(SerialBT.read());
		}
#endif

		LCD::heartbeat();
	}
}
