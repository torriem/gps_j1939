#include "bnorvc.h"
#include "globals.h"

BNORVC::BNORVC() {
	current_rvc = 0;
	for (int i=0; i < BNO_MAX_LOOKBACK; i++) {
		rvc_data[i].yaw = 400; //mark invalid
	}

}

void BNORVC::set_uart(Stream *uart) {
	current_rvc = 0;
	for (int i=0; i < BNO_MAX_LOOKBACK; i++) {
		rvc_data[i].yaw = 400; //mark invalid
	}
	//expect stream to already be set up.
	rvc.begin(uart);

	imu_timer = 0;
}

void BNORVC::process_data(void) {
	if(rvc.read(&rvc_data[current_rvc]) ) {
		current_rvc = (current_rvc + 1) % BNO_MAX_LOOKBACK;
		imu_timer = 0;
	} else if (imu_timer > 12) {
		//if 12 ms has elapsed, IMU must have gone silent
		imu_timer = 0;
		rvc_data[current_rvc].yaw = 400; //mark invalid
		current_rvc = (current_rvc + 1) % BNO_MAX_LOOKBACK;
	}
}

float BNORVC::get_roll(int lookback_ms) {
	int old_rvc = (BNO_MAX_LOOKBACK - lookback_ms / 10 + current_rvc) % BNO_MAX_LOOKBACK;

	float scale = (imu_reverse ? -1 : 1);

	if (rvc_data[old_rvc].yaw < 400) {
		if(imu_use_pitch) {
			return (rvc_data[old_rvc].pitch + imu_roll_offset) * scale;
		} else {
			return (rvc_data[old_rvc].roll + imu_roll_offset) * scale;
		}
	} else {
		return 400;
	}
}

float BNORVC::get_pitch(int lookback_ms) {
	int old_rvc = (BNO_MAX_LOOKBACK - lookback_ms / 10 + current_rvc) % BNO_MAX_LOOKBACK;

	float scale = (imu_reverse ? -1 : 1);

	if (rvc_data[old_rvc].yaw < 400) {
		if(imu_use_pitch) {
			return (rvc_data[old_rvc].roll + imu_roll_offset) * scale;
		} else {
			return (rvc_data[old_rvc].pitch + imu_roll_offset) * scale;
		}
	} else {
		return 400;
	}
}

float BNORVC::get_yaw(int lookback_ms) {
	int old_rvc = (BNO_MAX_LOOKBACK - lookback_ms / 10 + current_rvc) % BNO_MAX_LOOKBACK;

	float scale = (imu_reverse ? -1 : 1);

	if (rvc_data[old_rvc].yaw < 400) {
		return rvc_data[old_rvc].yaw * scale;
	} else {
		return 400;
	}
}


