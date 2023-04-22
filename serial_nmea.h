#ifndef __SERIAL_NMEA_H__
#define __SERIAL_NMEA_H__

#include <Arduino.h>

typedef void (*sn_process_imu)(void);

class SerialNMEA {
protected:
	const char *name;
	int id;

	// Interval based on the base GPS NMEA rate.
	// If interval is 1, emit every GPS position.
	// 2 is every other, 3 is every 3rd, etc.
	int gga_interval;
	int vtg_interval;
	int rmc_interval;
	int cfg_interval;

	bool imu_corrected;

	int gga_count;
	int vtg_count;
	int rmc_count;
	int cfg_count;

	Stream *dest;

public:
	SerialNMEA(int id_no, const char *name, Stream *stream);

	void set_stream (Stream *which_stream) { dest = which_stream; };
	void set_corrected (bool is_corrected) { imu_corrected = is_corrected; }
	void set_gga_interval (int interval) {gga_interval = interval;}
	void set_vtg_interval (int interval) {vtg_interval = interval;}
	void set_rmc_interval (int interval) {rmc_interval = interval;}
	void set_cfg_interval (int interval) {cfg_interval = interval;}

	bool get_corrected(void) { return imu_corrected; }
	int get_gga_interval(void) { return gga_interval; }
	int get_vtg_interval(void) { return vtg_interval; }
	int get_rmc_interval(void) { return rmc_interval; }
	int get_cfg_interval(void) { return cfg_interval; }
	//possibly rmc, PANDA

	const char *get_name(void) { return name; }
	void set_name(const char *the_name) {name = the_name;}
	int get_id(void) { return id; }
	void set_id(int the_id) { id = the_id; }
	bool is_active(void) { return ((dest != NULL) && (gga_interval || vtg_interval || rmc_interval || cfg_interval)); }
	bool is_active_gga(void) { return ((dest != NULL) && gga_interval); }
	bool is_active_vtg(void) { return ((dest != NULL) && vtg_interval); }
	bool is_active_rmc(void) { return ((dest != NULL) && rmc_interval); }
	bool is_active_cfg(void) { return ((dest != NULL) && cfg_interval); }

	
	//transmit NMEA sentences as configured
	void send_position(sn_process_imu process_imu = NULL);
	void send_string(const char *what, int length) {
		if (dest) dest->write(what, length);
	}

};

#endif
