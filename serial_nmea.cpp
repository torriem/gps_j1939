#include "serial_nmea.h"
#include "globals.h"
#include "nmea_gga.h"
#include "nmea_vtg.h"


SerialNMEA::SerialNMEA(int id_no, const char *the_name, Stream *stream) {
	use_gga = false;
	use_vtg = false;
	use_cfg = false;

	imu_corrected = false;
	dest = stream;
	id = id_no;
	name = the_name;
}

void SerialNMEA::send_position (void) {
	const char *msg;

	if (!dest) return;

	if (this->use_gga) {
		if (this->imu_corrected) {
			msg = nmea_gga.get_nmea();
		} else {
			msg = nmea_gga_uncorrected.get_nmea();
		}
		dest->write(msg,strlen(msg));
	}

	if (this->use_vtg) {
		msg = nmea_vtg.get_nmea();
		dest->write(msg,strlen(msg));
	}
}


