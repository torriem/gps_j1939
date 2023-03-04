#include "can.h"
#include "whichteensy.h"
#include "canframe.h"
#include "globals.h"
#if defined(TEENSY)
#   include <FlexCAN_T4.h>
//else if ESP or Due, include libraries
#endif

#ifndef TEENSY
#include <elapsedMillis.h>
#endif

static elapsedMillis last_can; // for periodic CAN messages
static FlexCAN_T4<CAN1, RX_SIZE_1024, TX_SIZE_1024> Can0;

static void j1939_decode(long ID, unsigned long* PGN, byte* priority, byte* src_addr, byte *dest_addr)
{
	/* decode j1939 fields from 29-bit CAN id */
	*src_addr = 255;
	*dest_addr = 255;

	*priority = (int)((ID & 0x1C000000) >> 26);

	*PGN = ID & 0x01FFFF00;
	*PGN = *PGN >> 8;

	ID = ID & 0x000000FF;
	*src_addr = (int)ID;

	/* decode dest_addr if a peer to peer message */
	if( (*PGN > 0 && *PGN <= 0xEFFF) ||
	    (*PGN > 0x10000 && *PGN <= 0x1EFFF) ) {
		*dest_addr = (int)(*PGN & 0xFF);
		*PGN = *PGN & 0x01FF00;
	}
}

static long j1939_encode(unsigned long pgn, byte priority, byte src_addr, byte dest_addr)
{

	long id;
	id = (priority & 0x07) << 26; //three bits only
	/* if a peer to peer message, encode dest_addr */
	if ((pgn > 0 && pgn <= 0xEFFF) ||
	    (pgn > 0x10000 && pgn <= 0x1efff)) {
	    	pgn = pgn & 0x01ff00;
		pgn = pgn | dest_addr;
	}
	id = id | (pgn << 8);
	id = id | src_addr;

	return id;
}

static inline void send_message(CANFrame &msg) {
	Can0.write(msg);
}

namespace CAN {
	void setup() {
		//Teensy FlexCAN_T4 setup
		Can0.begin();
		Can0.setClock(CLK_60MHz);
		Can0.setBaudRate(250000);
		Can0.setMaxMB(32);
		Can0.enableFIFO();
		//Can0.enableFIFOInterrupt();
	}

	void send_position() {
		CANFrame msg; //for generating gps messages

		msg.set_extended(true);
		msg.set_length(8); //all our messages are going to be 8 bytes

		/* Industry Standard messages */

		//gps position
		//create pgn 65267, priority 3, source 28, dest 2555
		msg.set_id(j1939_encode(65267, 3, 28, 255));
		msg.get_data()->uint32[0] = (gps_latitude + 210.0) * 10000000;
		msg.get_data()->uint32[1] = (gps_longitude + 210.0) * 10000000;
		send_message(msg);

		//PGN 65254, priority 3, src 28, dest 255
		//date and time
		msg.set_id(j1939_encode(65254,3,28,255));
		msg.get_data()->uint64 = gps_j1939_datetime;
		send_message(msg);

		// vehicle direction and speed
		// heading is uint16[0] / 128.0
		// speed is uint16[1] / 256.0 for km/h
		// pitch is uint16[2] / 128.0 - 200.0 for angle, pos is climbing, neg is sinking
		// altitude is uint16[3] / 0.125 - 2500 for metres.
		//vehicle direction and speed
		//pgn 65256, priority 3, src 28, dest 255
		msg.set_id(j1939_encode(65256,3,28,255));
		msg.get_data()->uint16[0] = gps_heading * 128;
		msg.get_data()->uint16[1] = gps_speed * 256;
		msg.get_data()->uint16[2] = 200 * 128; //not sure how critical vehicle pitch is
		msg.get_data()->uint16[3] = (gps_altitude + 2500) * 8;
		send_message(msg);

		/*Proprietary messages */

		//PGN 65535, first byte 51, priority 3, src 28, dest 255
		//GPS Status message
		msg.set_id(j1939_encode(65535,3,28,255));
		msg.get_data()->uint64 = 0xff191987ff020351;
		send_message(msg);

		//PGN 65535, first byte 52, priority 3, src 28, dest 255
		//GPS satellites used message
		msg.set_id(j1939_encode(65535,3,28,255));
		msg.get_data()->uint64 = 0x1fd47d4260a10552; //little endian
		send_message(msg);

		//PGN 65535, first byte 53, priority 6, src 28, dest 255
		//Differential receiver status
		msg.set_id(j1939_encode(65535,3,28,255));
		msg.get_data()->uint64 = 0x544110404a640153; //little endian

		if (gps_mode == 4) //RTK
			msg.get_data()->bytes[3] = 0x7a;
		else if (gps_mode == 5) //float
			msg.get_data()->bytes[3] = 0x66;
		else {

			//if no RTK, indicate that we have GPS, but low quality
			msg.get_data()->bytes[3] = 0x45;
		}

		send_message(msg);

		msg.set_id(j1939_encode(65535,3,28,255));
		msg.get_data()->uint64 = 0xffffffffffc000a0;
		send_message(msg);

		//suspect HDOP, VDOP are in this message
		msg.set_id(j1939_encode(65535,3,28,255));
		msg.get_data()->uint64 = 0x033020b90a000054;
		send_message(msg);

		//PGN 65535, first byte 0xe1, priority 2, src 28, dest 255
		//TCM pitch, roll, etc.
		msg.set_id(j1939_encode(65535,3,28,255));
		msg.get_data()->uint16[0] = 0xdfe1;
		msg.get_data()->uint16[1] = (gps_roll + 200) * 128.0;
		msg.get_data()->uint16[2] = (gps_yawrate + 200) * 128.0;
		//msg.get_data()->uint16[3] = 200 * 128; //probably vehicle pitch but not req'd
		msg.get_data()->uint16[3] = 0xfa00;
		send_message(msg);

		//not sure about this one! TCM message
		msg.set_id(j1939_encode(65535,3,28,255));
		msg.get_data()->uint64 = 0xf1ff1dfcffffffe3;
		send_message(msg);

		if (last_can >= 1000) {
			//unknown
			msg.set_id(j1939_encode(65535,6,28,255));
			msg.get_data()->uint64 = 0xffffff11fc08fde0; //should be once a second.
			//msg.get_data()->uint64 = 0xfffffffff00000e0;
			send_message(msg);

			msg.set_id(j1939_encode(61184,5,28,128));
			msg.get_data()->uint64 = 0x0000000000c3003f;
			msg.set_length(3);
			send_message(msg);

			msg.set_id(j1939_encode(61184,5,28,128));
			msg.get_data()->uint64 = 0xff05f100ef0000f1;
			msg.set_length(8);
			send_message(msg);

			msg.set_id(j1939_encode(61184,5,28,128));
			msg.get_data()->uint64 = 0x00000000003b003f;
			msg.set_length(3);
			send_message(msg);

			msg.set_length(8);

			// serial number of GPS Unknown frequency
			msg.set_id(j1939_encode(60928, 6, 28, 255));
			msg.get_data()->uint64 = 0x800017000421e240;
			//msg.get_data()->uint64 = 0x8000170004333020;
			send_message(msg);

			last_can = 0;
		}

	}

	void send_no_position() {
		CANFrame msg; //for generating gps messages
		msg.set_extended(true);
		msg.set_length(8); //all our messages are going to be 8 bytes

		/* standard messages */
		//emit null position messages to keep system happy
		msg.set_id(j1939_encode(65267, 3, 28, 255));
		msg.get_data()->uint64 = 0xffffffffffffffff; //unknown GPS position
		send_message(msg);

		msg.set_id(j1939_encode(65254,3,28,255));
		msg.get_data()->uint64 = 0xffff3ffd00000000; // unknown TIME
		send_message(msg);

		msg.set_id(j1939_encode(65256,3,28,255));
		msg.get_data()->uint64 = 0xffff6400ffffffff; // unknown attitude
		send_message(msg);

		/* proprietary messages */
		msg.set_id(j1939_encode(65535,3,28,255));
		msg.get_data()->uint64 = 0x0000000000000151; // no GPS
		send_message(msg);

		msg.set_id(j1939_encode(65535,3,28,255));
		msg.get_data()->uint64 = 0xffffff0000000052; //no GPS
		send_message(msg);

		msg.set_id(j1939_encode(65535,3,28,255));
		msg.get_data()->uint64 = 0x5441104040640153; //no GPS
		//msg.get_data()->uint64 = 0x5441104000000053; //no GPS
		send_message(msg);

		msg.set_id(j1939_encode(65535,3,28,255));
		msg.get_data()->uint64 = 0xffffffffffc000a0;
		send_message(msg);

		msg.set_id(j1939_encode(65535,3,28,255));
		msg.get_data()->uint64 = 0x3757b2801e000054; //no GPS
		send_message(msg);

		msg.set_id(j1939_encode(65535,3,28,255));
		msg.get_data()->uint64 = 0xfb0000fc00ffffe3; //unintialized TCM
		send_message(msg);

		msg.set_id(j1939_encode(65535,3,28,255));
		msg.get_data()->uint64 = 0x00ffffffffffe0e1; // unintialized TCM
		send_message(msg);


		if ( last_can >= 1000) {
			msg.set_id(j1939_encode(65535,3,28,255));
			msg.get_data()->uint64 = 0xfffffffff00000e0; //unintialized TCM
			send_message(msg);

			msg.set_id(j1939_encode(61184,5,28,128));
			msg.get_data()->uint64 = 0x0000000000c3003f;
			msg.set_length(3);
			send_message(msg);

			msg.set_id(j1939_encode(61184,5,28,128));
			msg.get_data()->uint64 = 0xff05f100ef0000f1;
			send_message(msg);

			msg.set_id(j1939_encode(61184,5,28,128));
			msg.get_data()->uint64 = 0x00000000003b003f;
			msg.set_length(3);
			send_message(msg);

			msg.set_length(8);

			// serial number of GPS not sure frequency
			msg.set_id(j1939_encode(60928, 6, 28, 255));
			msg.get_data()->uint64 = 0x800017000421e240;
			//msg.get_data()->uint64 = 0x8000170004333020;
			send_message(msg);
		}
	}
}


