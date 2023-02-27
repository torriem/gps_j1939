#ifndef _IMUBASE_H__
#define _IMUBASE_H__

class IMUBase {
public:
	IMUBase() { }

	virtual void process_data(void) = 0;
	virtual float get_roll(int lookback_ms) = 0;
	virtual float get_yaw(int lookback_ms) = 0;
	virtual float get_pitch(int lookback_ms) = 0;
};

#endif
