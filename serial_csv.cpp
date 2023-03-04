#include "serial_csv.h"
#include "globals.h"
#include "imubase.h"
#include <elapsedMillis.h>

char csv_buffer[256];
char roll_buffer[10];

/* this is a temporary debug printer */
void SerialCSV::send_position(IMUBase &imu, csv_process_imu process_imu) {
	char *buf;
	elapsedMillis t=0;
	if (!dest) return;

	snprintf(csv_buffer, 128,
	         "%s,%.7f,%.7f,%.2f,%.7f,%.7f,%.2f,%d,%.2f,%.2f,%.2f,%.2f,",
		 gps_fix_time,
		 gps_longitude,
		 gps_latitude,
		 gps_altitude,
		 gps_orig_longitude,
		 gps_orig_latitude,
		 gps_orig_altitude,
		 gps_mode,
		 gps_speed,
		 gps_heading,
		 gps_roll,
		 gps_yawrate);
	dest->write(csv_buffer,strlen(csv_buffer));
	
	if(process_imu) process_imu();
	
	buf = csv_buffer;
	int left = 256;

	//print all previous IMU positions, starting with
	//most recent.
	int oldest = imu.get_oldest_time();
	int s;
	float roll;

	/* this is too slow. why?*/
	for (int i=0; i < oldest ; i+=10) {
		roll = imu.get_roll(i);
		s = snprintf(buf,left, "%.2f,", roll);
		buf += s;
		left -= s;
		if (!left) break;
	}
	dest->println(csv_buffer);
	if(process_imu) process_imu();
}