#ifndef __BNORVC_H__
#define __BNORVC_H__

#include "imubase.h"
#include <Arduino.h>
#include <Adafruit_BNO08x_RVC.h>
#ifndef TEENSY
#include <elapsedMillis.h>
#endif

#define BNO_MAX_LOOKBACK 20
#define BNO_MS 10 //10 ms per message

class BNORVC: public IMUBase {
protected:
	Adafruit_BNO08x_RVC rvc;
	BNO08x_RVC_Data rvc_data[BNO_MAX_LOOKBACK];
	int current_rvc = 0;
	elapsedMillis imu_timer;
	Stream *uart;

public:
	BNORVC();

	void set_uart(Stream *the_uart);
	void process_data(void);
	float get_roll (int lookback_ms);
	float get_pitch (int lookback_ms);
	float get_yaw (int lookback_ms);
	int get_oldest_time(void);
};

#endif
