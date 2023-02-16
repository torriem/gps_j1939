/*
    This program is intended to run on either a Teensy 4.x.
    Can optionally use a TFT screen for a readout and debugging.

    This program does two things.  First it can enable steering with
    autotrac using only WAAS.  It does this by changing a CAN message
    to indicate SF1 is available even when it is not.

    Second, it can receive GPS position information from a SkyTraq
    PX1172RH dual-antenna GPS board. After calculating the GPS position
    on the ground, corrected for the roll of the tractor, it
    transmits this GPS position on the CAN bus using standard j1939
    PGNs, in place of any existing GPS messages that were intercepted.

    On Teensy requires the FlexCAN_T4 library, which ships with the
    TeensyDuino Arduino IDE add-on.  Also if the TFT display is used,
    requires the Adafruit TFT library which should be a part of the
    Arduino IDE already.

    This project is licensed under the GPL v3 or greater.  Please see
    https://www.gnu.org/licenses/gpl-faq.en.html for more information.

    /Copyright 2018-2022 Michael Torrie
    torriem@gmail.com
 */

#include "whichteensy.h"
#ifndef TEENSY
#  error This sketch requires a Teensy 4.x microcontroller
#endif

#include <math.h>
#include "canframe.h" //also defines TEENSY
#include "circle_generator.h"
#include "static_position.h"
#include "px1172rh.h"
#include <FlexCAN_T4.h>

#define TEENSY_TFT //comment out if don't want any screen.
#ifdef TEENSY_TFT
#  include <SD.h>
#  include <SPI.h>
#  include <ST7735_t3.h> // Hardware-specific library
#  include <ST7789_t3.h> // Hardware-specific library
#  include <st7735_t3_font_Arial.h>
#  include "ST7735_t3_font_ArialBold.h"
#  include "font_LiberationMono.h"
#  define TFT_RST    9   // chip reset
#  define TFT_DC     8   // tells the display if you're sending data (D) or commands (C)   --> WR pin on TFT
#  define TFT_MOSI   11  // Data out    (SPI standard)
#  define TFT_SCLK   13  // Clock out   (SPI standard)
#  define TFT_CS     10  // chip select (SPI standard)
#  define SD_CS     7
#  define rgb(R,G,B) (int)(R*31)*64*32 + (int)(G*63)*32 + (int)(B*31)
#endif

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

double antenna_forward=0; //antenna this far ahead of axle
double antenna_height=120 * INCHES; //inches above ground
double antenna_right=26.5 * INCHES; //for double antenna, how far away the right-most antenna is from the from center
#define GPS_TIMEOUT 400 //after 200ms of no GPS position, show "No GPS" on monitor

uint8_t gps_source = GPS_PX1172RH;
uint8_t virtual_source = VIRTUAL_NONE;

//int8_t monitor_can = -1;
int8_t monitor_can = 0;

//external GPS source variables
double autosteer_lat=0;
double autosteer_orig_lat=0;
double autosteer_lon=0;
double autosteer_orig_lon=0;
double autosteer_heading = 0;
double autosteer_roll = 0;
double autosteer_yawrate = 0;
double autosteer_speed = 0;
double autosteer_altitude = 0;
double autosteer_orig_altitude = 0;
uint64_t autosteer_datetime = 0x7d7d24300c00026c;

int autosteer_source=0; //0 = inadequate, 1=WAAS, 2 = SF1 or higher, 3 = External
long autosteer_lastext=0;
char autosteer_mode='G';

unsigned long last_61184 = millis();

long can_messages_received = 0; // a sort of heartbeat on the CAN bus.
uint8_t spinner_state = 0;

static const char *spinner="-\\|/";

double tractor_lat = -400;
double tractor_lon = -400;

uint16_t override_speed = 0;

bool debug_messages=false;

FlexCAN_T4<CAN1, RX_SIZE_1024, TX_SIZE_1024> Can0;
FlexCAN_T4<CAN2, RX_SIZE_1024, TX_SIZE_1024> Can1;

#ifdef TEENSY_TFT
ST7789_t3 tft = ST7789_t3(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

int16_t tft_drawDouble(double floatNumber, int dp, int poX, int poY)
{
  char str[14];               // Array to contain decimal string
  uint8_t ptr = 0;            // Initialise pointer for array
  int8_t  digits = 1;         // Count the digits to avoid array overflow
  double rounding = 0.5;       // Round up down delta

  if (dp > 7) dp = 7; // Limit the size of decimal portion

  // Adjust the rounding value
  for (uint8_t i = 0; i < dp; ++i) rounding /= 10.0;

  if (floatNumber < -rounding)    // add sign, avoid adding - sign to 0.0!
  {
    str[ptr++] = '-'; // Negative number
    str[ptr] = 0; // Put a null in the array as a precaution
    digits = 0;   // Set digits to 0 to compensate so pointer value can be used later
    floatNumber = -floatNumber; // Make positive
  }

  floatNumber += rounding; // Round up or down

  // For error put ... in string and return (all TFT_ILI9341_ESP library fonts contain . character)
  if (floatNumber >= 2147483647) {
    strcpy(str, "...");
    //return drawString(str, poX, poY);
  }
  // No chance of overflow from here on

  // Get integer part
  unsigned long temp = (unsigned long)floatNumber;

  // Put integer part into array
  ltoa(temp, str + ptr, 10);

  // Find out where the null is to get the digit count loaded
  while ((uint8_t)str[ptr] != 0) ptr++; // Move the pointer along
  digits += ptr;                  // Count the digits

  str[ptr++] = '.'; // Add decimal point
  str[ptr] = '0';   // Add a dummy zero
  str[ptr + 1] = 0; // Add a null but don't increment pointer so it can be overwritten

  // Get the decimal portion
  floatNumber = floatNumber - temp;

  // Get decimal digits one by one and put in array
  // Limit digit count so we don't get a false sense of resolution
  uint8_t i = 0;
  while ((i < dp) && (digits < 12)) // while (i < dp) for no limit but array size must be increased
  {
    i++;
    floatNumber *= 10;       // for the next decimal
    temp = floatNumber;      // get the decimal
    ltoa(temp, str + ptr, 10);
    ptr++; digits++;         // Increment pointer and digits count
    floatNumber -= temp;     // Remove that digit
  }

  // Finally we can plot the string and return pixel length
  return tft.drawString(str, poX, poY);
}
#endif

static inline void print_hex(uint8_t *data, int len) {
	char temp[4];
	for (int b=0;b < len; b++) {
		sprintf(temp, "%.2x ",data[b]);
		Serial.print(temp);
	}
	for (int b=0;b < len ; b++) {
		if ((data[b] < 32) || (data[b] > 126))
			Serial.print(".");
		else {
			sprintf(temp, "%c", data[b]);
			Serial.print(temp);
		}
	}
	Serial.println("");
}

void j1939_decode(long ID, unsigned long* PGN, byte* priority, byte* src_addr, byte *dest_addr)
{
	/* decode j1939 fields from 29-bit CAN id */
	*src_addr = 255;
	*dest_addr = 255;

	*priority = (int)((ID & 0x1C000000) >> 26);

	*PGN = ID & 0x00FFFF00;
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

long j1939_encode(unsigned long pgn, byte priority, byte src_addr, byte dest_addr)
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

/****
 Real work done below here
 ****/

void got_frame(CANFrame &frame, int which) {
	/*
	unsigned long PGN;
	byte priority;
	byte srcaddr;
	byte destaddr;
	bool send;

	j1939_decode(frame.get_id(), &PGN, &priority, &srcaddr, &destaddr);
	
	if (which == 0) {
		Can1.write(frame);
	} else {
		//transmit it out Can0
		Can0.write(frame);
	}
	*/
}

void teensy_got_frame(const CAN_message_t &orig_frame) {
	CANFrame frame = orig_frame;

	frame.seq=1;

	got_frame(frame, orig_frame.bus - 1);
}

void send_message(CANFrame &msg) {
	if (monitor_can < 0 ) return; //we don't know where to send it

	//only send it to the monitor
	if (monitor_can == 1 ) {
		Can1.write(msg);
	} else {
		Can0.write(msg);
	}
}

void send_nogps_messages(unsigned long t){
	CANFrame msg; //for generating gps messages
	msg.set_extended(true);
	msg.set_length(8); //all our messages are going to be 8 bytes

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

	msg.set_id(j1939_encode(65535,3,28,255));
	msg.get_data()->uint64 = 0xfffffffff00000e0; //unintialized TCM
	send_message(msg);

	if ( (t - last_61184) >= 1000) {
		last_61184 = t;
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

		// what is this pgn? serial number of GPS
		// what frequency? 
		msg.set_id(j1939_encode(60928, 6, 28, 255));
		msg.get_data()->uint64 = 0x800017000421e240;
		//msg.get_data()->uint64 = 0x8000170004333020;
		send_message(msg);
	}

}

void send_gps_messages() {
	CANFrame msg; //for generating gps messages
	autosteer_lastext = millis();
	autosteer_source = 3;

	msg.set_extended(true);
	msg.set_length(8); //all our messages are going to be 8 bytes

	//gps position
	//create pgn 65267, priority 3, source 28, dest 2555
	msg.set_id(j1939_encode(65267, 3, 28, 255));
	msg.get_data()->uint32[0] = (autosteer_lat + 210.0) * 10000000;
	msg.get_data()->uint32[1] = (autosteer_lon + 210.0) * 10000000;
	send_message(msg);

	//PGN 65254, priority 3, src 28, dest 255
	//date and time
	msg.set_id(j1939_encode(65254,3,28,255));
	msg.get_data()->uint64 = autosteer_datetime;
	send_message(msg);

	// vehicle direction and speed
	// heading is uint16[0] / 128.0
	// speed is uint16[1] / 256.0 for km/h
	// pitch is uint16[2] / 128.0 - 200.0 for angle, pos is climbing, neg is sinking
	// altitude is uint16[3] / 0.125 - 2500 for metres.
	//vehicle direction and speed
	//pgn 65256, priority 3, src 28, dest 255
	msg.set_id(j1939_encode(65256,3,28,255));
	msg.get_data()->uint16[0] = autosteer_heading * 128;
	msg.get_data()->uint16[1] = autosteer_speed * 256;
	msg.get_data()->uint16[2] = 200 * 128; //not sure how critical vehicle pitch is
	msg.get_data()->uint16[3] = (autosteer_altitude + 2500) * 8;
	send_message(msg);

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

	if (autosteer_mode == 'R')
		msg.get_data()->bytes[3] = 0x7a;
	else if (autosteer_mode == 'F')
		msg.get_data()->bytes[3] = 0x66;
	else {

		//if no RTK, show that we have GPS, but very poor quality
		//cannot really steer.
		msg.get_data()->bytes[3] = 0x40;
	}

	send_message(msg);

	msg.set_id(j1939_encode(65535,3,28,255));
	msg.get_data()->uint64 = 0xffffffffffc000a0;
	send_message(msg);

	//suspect HDOP, VDOP are in this message
	msg.set_id(j1939_encode(65535,3,28,255));
	msg.get_data()->uint64 = 0x033020b90a000054;
	send_message(msg);

	//Serial.print(autosteer_heading);
	//Serial.print(", ");
	//Serial.println(autosteer_yawrate);

	//PGN 65535, first byte 0xe1, priority 2, src 28, dest 255
	//TCM pitch, roll, etc.
	msg.set_id(j1939_encode(65535,3,28,255));
	msg.get_data()->uint16[0] = 0xdfe1;
	msg.get_data()->uint16[1] = (autosteer_roll + 200) * 128.0;
	msg.get_data()->uint16[2] = (autosteer_yawrate + 200) * 128.0;
	//msg.get_data()->uint16[3] = 200 * 128; //probably vehicle pitch on a 3000.
	msg.get_data()->uint16[3] = 0xfa00;
	send_message(msg);

	//not sure about this one! TCM message
	msg.set_id(j1939_encode(65535,3,28,255));
	msg.get_data()->uint64 = 0xf1ff1dfcffffffe3;
	send_message(msg);

	//TCM message.... investigate
	msg.set_id(j1939_encode(65535,6,28,255));
	msg.get_data()->uint64 = 0xffffff11fc08fde0; //should be once a second.
	//msg.get_data()->uint64 = 0xfffffffff00000e0; //from sf3000
	send_message(msg);


	if ((millis() - last_61184) >= 1000) {
		last_61184 = millis();

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

		// what is this pgn? serial number of GPS
		// what frequency? 
		msg.set_id(j1939_encode(60928, 6, 28, 255));
		msg.get_data()->uint64 = 0x800017000421e240;
		//msg.get_data()->uint64 = 0x8000170004333020;
		send_message(msg);

	}
}

void setup()
{
	debug_messages = false;
	Serial.begin(115200);
	delay(2000);
	Serial.println("gps_j1939");
	autosteer_source = 0;
	autosteer_lat = 0;
	autosteer_lon = 0;
	override_speed = 0;

	switch(gps_source) {
	case GPS_PX1172RH:
		setup_psti(send_gps_messages);
		break;
	}

	Serial3.addMemoryForRead(serial_buffer,1024);
	Serial3.begin(115200);

	//Teensy FlexCAN_T4 setup
	Can0.begin();
	Can0.setClock(CLK_60MHz);
	Can0.setBaudRate(250000);
	Can0.setMaxMB(32);
	Can0.enableFIFO();
	Can0.onReceive(teensy_got_frame);

	Can1.begin();
	Can1.setClock(CLK_60MHz);
	Can1.setBaudRate(250000);
	Can1.setMaxMB(32);
	Can1.enableFIFO();
	Can1.onReceive(teensy_got_frame);

	Can0.enableFIFOInterrupt();
	Can1.enableFIFOInterrupt();

#    ifdef TEENSY_TFT
	tft.init(240, 240);
	tft.setRotation(0);
	tft.fillScreen(0xffff);
	tft.setTextColor(ST77XX_RED,0xffff);
	tft.setCursor(5,217);
	tft.setFont(Arial_16_Bold);
	tft.println("Waiting for messages.");

	/*
	Sd2Card card;
	tft.setCursor(5,90);
	if(!card.init(SPI_HALF_SPEED, 7)) {
		tft.println("Could not find SDcard");
	} else {
		switch (card.type()) {
		case SD_CARD_TYPE_SD1:
			tft.println("SD1");
			break;

		case SD_CARD_TYPE_SD2:
			tft.println("SD2");
			break;

		case SD_CARD_TYPE_SDHC:
			tft.println("SDHC");
			break;

		default:
			tft.println("Unknown");

		}

	}
	*/
#    endif //TEENSY_TFT
}

void loop()
{
	char c;
	
	unsigned long last_time = millis();
	unsigned long t;

	// virtual position generators
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
		//perhaps check to see how long since we had our last external GPS reading.
		//if it's been a certain amount of time, revert to the tractor receiver

		t = millis();
		if ((t - autosteer_lastext) > GPS_TIMEOUT) {
			if (autosteer_source == 3) autosteer_source = 0;
			autosteer_lastext = t;
			send_nogps_messages(t);
		}

		if (t - last_time > 500) {
#ifdef TEENSY_TFT
			tft.setTextColor(rgb(0,0,0), 0xffff);
			tft.setFont(LiberationMono_20);
			tft.setCursor(5,115);
			tft.print(can_messages_received);
			tft.print(" ");
			tft.print(spinner[spinner_state]);
			tft.println(" ");
			spinner_state=(spinner_state+1) % 4;
#endif

			if (autosteer_lat) {
#ifdef TEENSY_TFT
				tft.setTextColor(rgb(0,0,0), 0xffff);
				tft.setFont(LiberationMono_20);
				tft_drawDouble(autosteer_lat,7,5,5);
				tft_drawDouble(autosteer_lon,7,5,30);
				tft.setTextColor(0,0xffff);
				tft_drawDouble(autosteer_roll,2,5,55);
				tft_drawDouble(autosteer_heading,2,5,80);
				tft.setCursor(5,217);
				tft.setFont(Arial_16_Bold);

				switch (autosteer_source) {
				case 0: //inadequate GPS
					tft.setTextColor(ST77XX_RED,0xffff);
					tft.println("No GPS.                     ");
					break;
				case 1: //WAAS or lower
					tft.setTextColor(ST77XX_BLUE,0xffff);
					tft.println("Steer with SF WAAS         ");
					break;
				case 2: //SF1 or higher, internal
					tft.setTextColor(ST77XX_BLACK,0xffff);
					tft.println("Steer with SF SF1          ");
					break;

				case 3:
					if (autosteer_mode == 'R') {
						tft.setTextColor(ST77XX_GREEN,0xffff);
						tft.println("Steer with ext RTK            ");
					} else if (autosteer_mode == 'F') {
						tft.setTextColor(ST77XX_RED,0xffff);
						tft.println("Steer with ext Float RTK      ");
					} else {
						tft.setTextColor(ST77XX_RED,0xffff);
						tft.println("Poor GPS; no steer             ");
					}
					break;
				}
#else
				switch (autosteer_source) {
				case 0:
					Serial.println("Poor GPS, but steering.");
					break;
				case 1: //WAAS or lower
					Serial.println("Steer with WAAS");
					break;
				case 2: //SF1 or higher, internal
					Serial.println("Steer with SF1 or +");
					break;

				case 3:
					Serial.println("Steer with ext GPS ");
					break;
				}
				Serial.print(autosteer_lat,7);
				Serial.print(" ");
				Serial.println(autosteer_lon,7);
#endif
			}
#ifndef TEENSY_TFT
			else {
				//Serial.println("Waiting for data.");
			}
#endif

			last_time = t;
		}

		switch(virtual_source) {
		case VIRTUAL_CIRCLE:
			if (circlegen.circle_position(t, 200)) {
				autosteer_source = 3;
				send_gps_messages();
				autosteer_lastext = t;
			}
			if (circlegen.finished(t))
				virtual_source = VIRTUAL_NONE;
			break;
		case VIRTUAL_STATIC:
			if (staticpos.static_position(t, 200)) {
				autosteer_source = 3;
				send_gps_messages();
				autosteer_lastext = t;
			}
			if (staticpos.finished(t))
				virtual_source = VIRTUAL_NONE;
			break;
		}

		//now process serial bytes that have accumulated
		while(Serial3.available()) {
			c = Serial3.read();

			if (virtual_source) {
				/* ignore real GPS while virtual positions are
				   being used. */
				continue;
			}

			switch(gps_source) {
			case GPS_PX1172RH:
				psti_process(c);
				break;
			case GPS_NMEA_BNO:
				//TODO
				break;
			}
		}
	}
}
