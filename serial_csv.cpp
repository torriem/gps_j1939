#include "serial_csv.h"
#include "globals.h"
#include "imubase.h"
#include <elapsedMillis.h>

char csv_buffer[256];
char roll_buffer[10];

/* this is a temporary debug printer */
void SerialCSV::send_position(IMUBase &imu, csv_process_imu process_imu) {
	char *buf;
	if (!dest) return;

	int left = 256 - snprintf(csv_buffer, 128,
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

	//print all previous IMU positions, starting with
	//most recent.
	int s;
	float roll;

	/* this is too slow. why?*/
	for (int i=0; i < 20 ; i++) {
		s = snprintf(buf,left, "%.2f,", imu_roll_buffer[i]);
		buf += s;
		left -= s;
		if (left < 1) break;
	}
	dest->println(csv_buffer);
	if(process_imu) process_imu();
}
