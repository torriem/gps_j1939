#ifndef __SERIAL_CSV_H__
#define __SERIAL_CSV_H__

#include <Arduino.h>
#include "bnorvc.h"

typedef void (*csv_process_imu)(void);

class SerialCSV {
protected:
	Stream *dest;

public:
	SerialCSV(Stream *dest_stream = NULL) { dest = dest_stream; }
	void set_stream(Stream *dest_stream) { dest = dest_stream; }

	void send_position(IMUBase &imu, csv_process_imu process_imu = NULL);

};

#endif
