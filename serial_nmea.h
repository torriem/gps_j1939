#ifndef __SERIAL_NMEA_H__
#define __SERIAL_NMEA_H__

#include <Arduino.h>

class SerialNMEA {
protected:
	const char *name;
	int id;

	bool use_gga;
	bool use_vtg;
	bool use_cfg;
	bool imu_corrected;

	Stream *dest;

	static constexpr char *gga_str = "gga";
	static constexpr char *vtg_str = "vtg";
	static constexpr char *cfg_str = "cfg";
	static constexpr char *none = "";

public:
	SerialNMEA(int id_no, const char *name, Stream *stream);

	void set_stream (Stream *which_stream) { dest = which_stream; };
	void set_corrected (bool is_corrected) { imu_corrected = is_corrected; }
	void send_gga (bool use_it) {use_gga = use_it;}
	void send_vtg (bool use_it) {use_vtg = use_it;}
	void send_cfg (bool use_it) {use_cfg = use_it;}

	const char *get_gga(void) { if (use_gga) return gga_str; else return none;}
	const char *get_vtg(void) { if (use_vtg) return vtg_str; else return none;}
	const char *get_cfg(void) { if (use_cfg) return cfg_str; else return none;}
	//possibly rmc, PANDA

	const char *get_name(void) { return name; }
	void set_name(const char *the_name) {name = the_name;}
	int get_id(void) { return id; }
	void set_id(int the_id) { id = the_id; }
	bool is_active(void) { return ((dest != NULL) && (use_gga || use_vtg || use_cfg)); }
	bool is_active_gga(void) { return ((dest != NULL) && use_gga); }
	bool is_active_vtg(void) { return ((dest != NULL) && use_vtg); }
	bool is_active_cfg(void) { return ((dest != NULL) && use_cfg); }

	
	//transmit NMEA sentences as configured
	void send_position(void);
	void send_string(const char *what, int length) {
		if (dest) dest->write(what, length);
	}

};

#endif
