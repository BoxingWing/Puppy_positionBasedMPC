#ifndef _JOYSTICK_RASPI_H_
#define _JOYSTICK_RASPI_H_
#include "stdint.h"

/*
extern int joy_fd; // handle of the joystick
extern unsigned char NumOfAxix; // axis number
extern unsigned char NumOfButton; // button number
extern char joystick_name[1024];
extern struct js_event js; // event record of the joystick
extern short int value;
extern unsigned char axisbuttonNumber;
extern unsigned char ButtonState;
extern unsigned char AxisState;*/

void joystickSetup();
void eventRead();
int returnJSInfo(int16_t * valueReturn, int16_t * numberReturn);
unsigned char returnButtonState();
unsigned char returnAxisState();

#endif
