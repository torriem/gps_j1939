#include "bnorvc.h"
#include "globals.h"

BNORVC::BNORVC() {
	current_rvc = 0;
	for (int i=0; i < BNO_MAX_LOOKBACK; i++) {
		rvc_data[i].yaw = 400; //mark invalid
	}

}

void BNORVC::set_uart(Stream *which_uart) {
	current_rvc = 0;
	uart = which_uart;

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
	} else if (imu_timer > 100) {
		//if 100 ms has elapsed, IMU must have gone silent
		//don't want to cut this too short because messages could
		//be in the buffer
		imu_timer = 0;
		rvc_data[current_rvc].yaw = 400; //mark invalid
		current_rvc = (current_rvc + 1) % BNO_MAX_LOOKBACK;
	}

	//now read any messages if we've fallen behind a bit
	while(uart->available()) {
		if(rvc.read(&rvc_data[current_rvc]) ) {
			current_rvc = (current_rvc + 1) % BNO_MAX_LOOKBACK;
			imu_timer = 0;
		}
	}
}

float BNORVC::get_roll_from_index(int lookback_index){
	/* use a time slot index instead of ms.  Lookback_index is
	 * lookback_ms / 10
	 */

	float roll;

	int buffer_index = (BNO_MAX_LOOKBACK + lookback_index) % BNO_MAX_LOOKBACK;

	float scale = (imu_reverse ? -1 : 1);

	if (rvc_data[buffer_index].yaw < 400) {
		if(imu_use_pitch) {
			roll = (rvc_data[buffer_index].pitch + imu_roll_offset) * scale;
		} else {
			roll = (rvc_data[buffer_index].roll + imu_roll_offset) * scale;
		}
		return roll;
	} else {
		return 400;
	}


}

float BNORVC::get_roll_ave(int lookback_ms, int qave = 3) {
	/* calculate the average roll value across the qave window.
	 * qave is discrete readings.  Should always be odd, and the
	 * lookback_ms value should be in the buffer such that the
	 * entire required window of values are available.  If not,
	 * the average won't cover the desired time.
	 */

	float roll_sum;
	float roll_reading;

	int offset = (lookback_ms / 10);
	int start, end;

	//make sure the window is inside the buffer. Note that
	//end is the youngest time and start is the oldest
	end = offset + qave / 2;
	start = offset - qave / 2;
	if (end > BNO_MAX_LOOKBACK) {
		end = BNO_MAX_LOOKBACK;
	}

	if (start < 0) {
		start = 0;
	}
	
	int i;
	int count=0;
	roll_sum = 0;

	for (i=start; i < end; i++) {
		roll_reading = get_roll_from_index(i);
		if (roll_reading < 400) {
			count++;
			roll_sum += roll_reading;
		}
	}
	if(count) {
		//return the average of the valid readings in the window
		return roll_sum / count;
	}
	
	//no valid readings in the window, so return invalid.
	return 400;
			
}

float BNORVC::get_roll(int lookback_ms) {
	return get_roll_ave(lookback_ms, 3);
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

int BNORVC::get_oldest_time(void) {
	return BNO_MAX_LOOKBACK * 10;
}

