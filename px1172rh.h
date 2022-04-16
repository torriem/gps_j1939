#ifndef __PX1172RH_H__
#define __PX1172RH_H__

extern double antenna_forward;
extern double antenna_height;
extern double antenna_left;

extern double autosteer_lat;
extern double autosteer_lon;
extern double autosteer_heading;
extern double autosteer_roll;
extern double autosteer_yawrate;
extern double autosteer_speed;
extern double autosteer_altitude;
extern char autosteer_mode;

bool psti_process(char c);

#endif


