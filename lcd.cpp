#include "lcd.h"
#include "globals.h"

#include "whichteensy.h"
#ifndef TEENSY
#include <elapsedMillis.h>
#endif

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

static ST7789_t3 tft = ST7789_t3(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

static int16_t tft_drawDouble(double floatNumber, int dp, int poX, int poY) {
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

static uint8_t spinner_state = 0;
static const char *spinner="-\\|/";

static elapsedMillis last_heartbeat;
static elapsedMillis last_update;

namespace LCD {
	void setup() {
		last_heartbeat = 0;
		last_update = 0;
#ifdef TEENSY_TFT
		tft.init(240, 240);
		tft.setRotation(0);
		tft.fillScreen(0xffff);
		tft.setTextColor(ST77XX_RED,0xffff);
		tft.setCursor(5,217);
		tft.setFont(Arial_16_Bold);
		tft.println("Waiting for GPS.");
#endif
	}

	void heartbeat() {
#ifdef TEENSY_TFT
		if (last_heartbeat > 500) {
			last_heartbeat = 0;
			tft.setTextColor(rgb(0,0,0), 0xffff);
			tft.setFont(LiberationMono_20);
			tft.setCursor(5,115);
			tft.print(" ");
			tft.print(spinner[spinner_state]);
			tft.println(" ");
			spinner_state=(spinner_state+1) % 4;
		}
#endif
	}

	void new_position() {
#ifdef TEENSY_TFT
		if (last_update > 1000) {
			last_update = 0;
			tft.setTextColor(rgb(0,0,0), 0xffff);
			tft.setFont(LiberationMono_20);
			tft_drawDouble(gps_latitude,7,5,5);
			tft_drawDouble(gps_longitude,7,5,30);
			tft.setTextColor(0,0xffff);
			tft_drawDouble(gps_roll,2,5,55);
			tft_drawDouble(gps_heading,2,5,80);
			tft.setCursor(5,217);
			tft.setFont(Arial_16_Bold);

			switch (gps_mode) {
			case 5:
				tft.setTextColor(ST77XX_BLUE,0xffff);
				tft.println("Float RTK            ");
				break;
			case 4:
				tft.setTextColor(ST77XX_GREEN,0xffff);
				tft.println("RTK                   ");
				break;
			case 2:
				tft.setTextColor(ST77XX_RED,0xffff);
				tft.println("DGPS 3D               ");
				break;
			default:
				tft.setTextColor(ST77XX_RED,0xffff);
				tft.println("Poor or no GPS        ");
				break;
			}
		}
#endif
	}
}


