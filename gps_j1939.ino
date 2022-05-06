/*
    This program is intended to run on either a Teensy 4.x or an 
    Arduino Due.  Possibly can be made to run on any microcontroller
    that has two CAN interfaces and is reasonably fast, at least 100 
    Mhz.  As written the Teensy version requires a TFT screen. Will
    change that later.

    This program does two things.  First it can enable steering with
    autotrac using only WAAS.  It does this by changing a CAN message
    to indicate SF1 is available even when it is not.

    Second, it can receive GPS position information from a SkyTraq
    PX1172RH dual-antenna GPS board. After calculating the GPS position
    on the ground, corrected for the roll of the tractor, it
    transmits this GPS position to autotrac in place of the onboard
    receiver.

    This project is licensed under the GPL v3 or greater.  Please see
    https://www.gnu.org/licenses/gpl-faq.en.html for more information.

    /Copyright 2018-2022 Michael Torrie
    torriem@gmail.com
 */

#include <math.h>
#include "canframe.h" //also defines TEENSY

//Plan to support different kinds of external GPS using modules
#define EXTGPS_PX1172RH 1 

#ifdef EXTGPS_PX1172RH
#  include "px1172rh.h"
#endif 

#ifdef TEENSY
#  include <FlexCAN_T4.h>
#  define TEENSY_TFT //comment out if don't want any screen.

#  ifdef TEENSY_TFT
#    include <SD.h>
#    include <SPI.h>
#    include <ST7735_t3.h> // Hardware-specific library
#    include <ST7789_t3.h> // Hardware-specific library
#    include <ST7735_t3_font_Arial.h>
#    include <ST7735_t3_font_ArialBold.h>
#    include "font_LiberationMono.h"
#    define TFT_RST    9   // chip reset
#    define TFT_DC     8   // tells the display if you're sending data (D) or commands (C)   --> WR pin on TFT
#    define TFT_MOSI   11  // Data out    (SPI standard)
#    define TFT_SCLK   13  // Clock out   (SPI standard)
#    define TFT_CS     10  // chip select (SPI standard)
#    define SD_CS     7
#    define rgb(R,G,B) (int)(R*31)*64*32 + (int)(G*63)*32 + (int)(B*31)
#  endif
#else
//#  include "variant.h" //PI among other things
#  include <due_can.h>
#  include <can_common.h>

#  define Serial SerialUSB

#endif

#define RADIANS(deg) deg * M_PI / 180.0
#define DEGREES(rad) rad * 180.0 / M_PI

#ifdef TEENSY
FlexCAN_T4<CAN1, RX_SIZE_1024, TX_SIZE_1024> Can0;
FlexCAN_T4<CAN2, RX_SIZE_1024, TX_SIZE_1024> Can1;
#ifdef TEENSY_TFT
ST7789_t3 tft = ST7789_t3(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
#endif
#endif

double antenna_forward=0; //antenna this far ahead of axle
double antenna_height=3.048; //meters above ground
double antenna_left=0.6731; //for double antenna, how far away from center is the left-most antenna
#define EXT_GPS_TIMEOUT 5000 //after 5 seconds of no external position, revert to internal.


//external GPS source variables
double autosteer_lat=0;
double autosteer_lon=0;
double autosteer_heading = 0;
double autosteer_roll = 0;
double autosteer_yawrate = 0;
double autosteer_speed = 0;
double autosteer_altitude = 0;
int autosteer_source=0; //0 = inadequate, 1=WAAS, 2 = SF1 or higher, 3 = External
long autosteer_lastext=0;
char autosteer_mode='G';

long can_messages_received = 0; // a sort of heartbeat on the CAN bus.
uint8_t spinner_state = 0;

static const char *spinner="-\\|/";

double tractor_lat = -400;
double tractor_lon = -400;

//Save some frames so we can modify them with injected information
CANFrame vehicle_position_frame;
CANFrame vehicle_dirspeed_frame;
CANFrame tcm_roll_frame;

uint8_t receiver_from;

uint16_t override_speed = 0;

bool debug_messages=false;

static inline void print_hex(uint8_t *data, int len) {
	char temp[4];
	if (debug_messages) {
		for (int b=0;b < len; b++) {
			sprintf(temp, "%.2x ",data[b]);
			Serial.print(temp);
		}
		Serial.println("");
	}
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
	id = (priority & 0x01c) << 26;
	id = id | (pgn << 8);
	id = id | src_addr;
	id = id | dest_addr << 8;

	return dest_addr;
}

/****
 Real work done below here
 ****/

void got_frame(CANFrame &frame, int which) {
	unsigned long PGN;
	byte priority;
	byte srcaddr;
	byte destaddr;
	bool send;

	send = true;

	j1939_decode(frame.get_id(), &PGN, &priority, &srcaddr, &destaddr);

	/* look for specific frames that we're interested in */
	if (srcaddr == 240) {
		switch(PGN) {
		case 65096: //vehicle wheel speed
			if(override_speed) {
				frame.get_data()->uint16[0] = override_speed;
				//override_speed = 0; // turn it  off now? How to do this properly?
			}
			break;
		}
	}
	if (srcaddr == 128) {
		switch(PGN) {
		case 65535:
			switch(frame.get_data()->bytes[0]){
			case 0x31: //maybe speed in here
				if (override_speed) {
					frame.get_data()->uint8[1] = (override_speed >> 8);
					frame.get_data()->uint8[2] = (override_speed & 0xff);
					Serial.println ("Overriding speed.");
				}
			}
		}
	}
	if (srcaddr == 28) {
		receiver_from = which; //remember which CAN interface the GPS receiver is on
		send=false;

		switch(PGN) {
		case 65535:
			switch(frame.get_data()->bytes[0]) {
			case 0x53: // GPS accuracy level and steering flag
				send=true;
				if (autosteer_source == 3) {
					//external GPS, RTK presumably

					//we could try setting to 0x70 and 0x7a
					frame.get_data()->bytes[4] = 0x40; //steering toggle should always be on
					frame.get_data()->bytes[3] = 0x7a; //RTK, accuracy 10
				} else {
					uint8_t signal_type;

					//ensure steering works when sf1 isn't present
					frame.get_data()->bytes[4] = 0x40; //steering toggle should always be on
					signal_type = frame.get_data()->bytes[3] >> 4;

					if (signal_type < 4) {
						if (signal_type > 1) {
							if(debug_messages) {
								Serial.println("Steer with differential signal.");
							}
							autosteer_source = 1;

							//accuracy of 3 is minimum for steering.
							//0x10 = Diff 0x20 = JDDiff, 0x30 = WAAS, 0x40 = sf1
							//frame->data.bytes[3] = 0x55; //0x5x=sf2
							//frame->data.bytes[3] = 0x75; //0x6x=rtk-x, 0x7x=rtk

							frame.get_data()->bytes[3] = 0x46; //SF1, accuracy 6
						} else {
							//no WAAS.  Performance is unlikely to be acceptable.
							autosteer_source = 0;
							if(debug_messages) {
								Serial.println("Steer with 3D+ signal.");
							}
							frame.get_data()->bytes[3] = 0x42; //SF1, accuracy 2
						}

					} else {
						autosteer_source = 2;
					}
				}

				//Serial.println("got 0x53");
				break;
			case 0xe1: //TCM information
				send=true;
				tcm_roll_frame = frame;
				autosteer_roll = frame.get_data()->uint16[1] / 128.0 - 200.0;
				autosteer_yawrate = frame.get_data()->uint16[2] / 128.0 - 200.0;
				//Serial.println("got 0xe1");
				break;
			default:
				if(destaddr == 255) {
					char firstbyte = frame.get_data()->bytes[0];
					switch(firstbyte) {
						case 0x51:
						case 0x52:
							//only 0x51, 0x52, 0x53, and 0xe1 are
							//necessary to steer. Ignore all others.
							send=true;
							break;
					}

				}
				break;
			}

			//dump out required messages for analysis.
			switch(frame.get_data()->bytes[0]) {
			case 0x51:
			case 0x52:
			case 0x53:
			case 0xe1:
				debug_messages=true;
				Serial.print(millis());
				Serial.print(" ");
				Serial.print(PGN);
				Serial.print(" ");
				print_hex(frame.get_data()->bytes, frame.get_length());
				debug_messages=false;
			}
			break;

		case 65267:
			send=true;
			can_messages_received ++; //heartbeat
			// vehicle position
			// latitude is uint32[0] / 10000000.0 - 210 for degres
			// longitude is uint32[0] / 10000000.0 - 210 for degrees
			vehicle_position_frame = frame;
			if (autosteer_source == 3) {
				//if we're using external GPS, don't send this frame on
				send = false;
			} else {
				//update the display
				autosteer_lat = frame.get_data()->uint32[0] / 10000000.0 - 210.0;
				autosteer_lon = frame.get_data()->uint32[1] / 10000000.0 - 210.0;
			}
			//Serial.println("got 65267");
			break;
			
		case 65256:
			send=true;
			// vehicle direction and speed
			// heading is uint16[0] / 128.0
			// speed is uint16[1] / 256.0 for km/h
			// pitch is uint16[2] / 128.0 - 200.0 for angle, pos is climbing, neg is sinking
			// altitude is uint16[3] / 0.125 - 2500 for metres.

			//autosteer_altitude = frame.get_data()->uint16[3] * 0.125 - 2500;

			if (autosteer_source == 3) {
				send = false;
			} else {
				// vehicle heading
				autosteer_heading = frame.get_data()->uint16[0] / 128.0;
			}
			
			vehicle_dirspeed_frame = frame;
			//Serial.println("got 65256");
			break;

		default:
			if(destaddr == 255 && PGN==65254) {
				send=true;
				debug_messages=true;
				Serial.print(millis());
				Serial.print(" ");
				Serial.print(PGN);
				Serial.print(" ");

				print_hex(frame.get_data()->bytes, frame.get_length());
				debug_messages=false;
			}
		}
	}

	//transmit frames if we're not collecting them
	if (!send) return;
	
	if (which == 0) {
#ifdef TEENSY
		Can1.write(frame);
		//Can1.events();
#else
		Can1.sendFrame(frame);
#endif
	} else {
		//transmit it out Can0
#ifdef TEENSY
		Can0.write(frame);
		//Can0.events();
#else
		Can0.sendFrame(frame);
#endif
	}
}

#ifdef TEENSY
void teensy_got_frame(const CAN_message_t &orig_frame) {
	CANFrame frame = orig_frame;

	frame.seq=1;

	got_frame(frame, orig_frame.bus - 1);
}

#else
void can0_got_frame(CAN_FRAME *orig_frame) {
	//Serial.print("0");
	//CANFrame *wrapper = frame;
	CANFrame frame = static_cast<CANFrame>(*orig_frame);
	got_frame(frame, 0);
}

void can1_got_frame(CAN_FRAME *orig_frame) {
	//Serial.print("<");
	//CANFrame *wrapper = frame;
	CANFrame frame = static_cast<CANFrame>(*orig_frame);
	got_frame(frame, 1);
}
#endif

void setup()
{
	debug_messages = false;
	Serial.begin(115200);
	delay(2000);
	Serial.println("Waas2SF1.");
	autosteer_source = 0;
	autosteer_lat = 0;
	autosteer_lon = 0;
	override_speed = 0;

#ifdef TEENSY

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

#ifdef TEENSY_TFT
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
#endif //TEENSY_TFT

#else
	Can0.begin(CAN_BPS_250K);
	Can1.begin(CAN_BPS_250K);

	for (int filter=0;filter <3; filter ++) {
		Can0.setRXFilter(0,0,true);
		Can1.setRXFilter(0,0,true);
	}

	Can0.attachCANInterrupt(can0_got_frame);
	Can1.attachCANInterrupt(can1_got_frame);
	Serial.println("Waiting for messages.");
#endif
}

void loop()
{
	char c;
	
	unsigned long last_time = millis();

	while(1) {
		//perhaps check to see how long since we had our last external GPS reading.
		//if it's been a certain amount of time, revert to the tractor receiver
		if (autosteer_source == 3 && millis() - autosteer_lastext > EXT_GPS_TIMEOUT) {
			autosteer_source = 0;
		}

		if (millis() - last_time > 1000) {
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

			//Serial.println(millis());
			if (autosteer_lat) {
#ifdef TEENSY_TFT
				tft.setTextColor(rgb(0,0,0), 0xffff);
				tft.setFont(LiberationMono_20);
				tft.drawDouble(autosteer_lat,7,5,5);
				tft.drawDouble(autosteer_lon,7,5,30);
				tft.setTextColor(0,0xffff);
				tft.drawDouble(autosteer_roll,2,5,55);
				tft.drawDouble(autosteer_heading,2,5,80);
				tft.setCursor(5,217);
				tft.setFont(Arial_16_Bold);
				switch (autosteer_source) {
				case 0: //inadequate GPS
					tft.setTextColor(ST77XX_RED,0xffff);
					tft.println("Poor GPS; but steering    ");
					break;
				case 1: //WAAS or lower
					tft.setTextColor(ST77XX_BLUE,0xffff);
					tft.println("Steer with WAAS           ");
					break;
				case 2: //SF1 or higher, internal
					tft.setTextColor(ST77XX_BLACK,0xffff);
					tft.println("Steer with SF1 or +        ");
					break;

				case 3:
					if (autosteer_mode == 'R') {
						tft.setTextColor(ST77XX_GREEN,0xffff);
						tft.println("Steer with ext RTK            ");
					} else {
						tft.setTextColor(ST77XX_RED,0xffff);
						tft.println("Steer with ext Float RTK      ");
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
				Serial.println("Waiting for data.");
			}
#endif

			last_time = millis();
		}

		while(Serial.available()) {
			c = Serial.read();

#ifdef EXTGPS_PX1172RH
			if (psti_process(c)) {
				autosteer_lastext = millis();
				autosteer_source = 3; //tell CAN proxy we're injecting GPS positions now
				//if external RTK, inject a CAN message with our position
				CANFrame msg;
				//copy a message we've already seen so the ID will be correct
				//we'll make them from scratch once we know this works.
				msg = vehicle_position_frame;
				msg.get_data()->uint32[0] = (autosteer_lat + 210.0) * 10000000;
				msg.get_data()->uint32[1] = (autosteer_lon + 210.0) * 10000000;

				if (receiver_from == 1 ) {
					Can0.write(msg);
				} else {
					Can1.write(msg);
				}

				//try overriding heading with our exact dual gps heading
				msg = vehicle_dirspeed_frame;
				msg.get_data()->uint16[0] = autosteer_heading / 100.0 * 128;

				//Serial.println(autosteer_speed);
				//msg.get_data()->uint16[1] = autosteer_speed * 256;

				//vehicle pitch
				//msg.get_data()->uint[2] = 200 * 128;

				msg.get_data()->uint16[3] = (autosteer_altitude / 1000.0 + 2500) * 8;

				if (receiver_from == 1 ) {
					Can0.write(msg);
				} else {
					Can1.write(msg);
				}

			}
#endif
		}
	}
}
