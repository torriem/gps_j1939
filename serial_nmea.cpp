#include "serial_nmea.h"
#include "serial_config.h"
#include "globals.h"
#include "nmea_gga.h"
#include "nmea_vtg.h"
#include "nmea_rmc.h"


SerialNMEA::SerialNMEA(int id_no, const char *the_name, Stream *stream) {
	gga_interval = 0;
	vtg_interval = 0;
	rmc_interval = 0;
	cfg_interval = 0;

	gga_count = 0;
	vtg_count = 0;
	rmc_count = 0;
	vtg_count = 0;

	imu_corrected = false;
	dest = stream;
	id = id_no;
	name = the_name;
}

//TODO: might need callback to process IMU messages if
//this takes too long to write everything to serial.
void SerialNMEA::send_position (sn_process_imu process_imu) {
	const char *msg;

	if (!dest) return;

	if (gga_interval) {
		gga_count++;
		if  (gga_count == gga_interval) {
			gga_count = 0;

			if (this->imu_corrected) {
				msg = nmea_gga.get_nmea();
			} else {
				msg = nmea_gga_uncorrected.get_nmea();
			}
			dest->write(msg,strlen(msg));
		}
	}

	if (process_imu) process_imu();

	if (vtg_interval) {
		vtg_count++;
		if (vtg_count == vtg_interval) {
			vtg_count = 0;

			msg = nmea_vtg.get_nmea();
			dest->write(msg,strlen(msg));
		}
	}

	if (process_imu) process_imu();

	if (rmc_interval) {
		rmc_count++;
		if (rmc_count == rmc_interval) {
			rmc_count = 0;
			msg = nmea_rmc.get_nmea();
			dest->write(msg,strlen(msg));
		}
	}

	if (process_imu) process_imu();

	if (cfg_interval) {
		cfg_count++;
		if (cfg_count == cfg_interval) {
			cfg_count = 0;
			msg = serial_config::get_nmea();
			dest->write(msg,strlen(msg));
		}
	}

	if (process_imu) process_imu();
}

