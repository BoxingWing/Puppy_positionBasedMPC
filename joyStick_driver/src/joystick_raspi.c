#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>
#include "joystick_raspi.h"

#define JOY_DEV "/dev/input/js0"

int joy_fd; // handle of the joystick
unsigned char NumOfAxix; // axis number
unsigned char NumOfButton; // button number
unsigned char ButtonState;
unsigned char AxisState;
char joystick_name[1024];
struct js_event js; // event record of the joystick
short int value[64];
unsigned char axisbuttonNumber[64];
int LoopCount;


void joystickSetup()
{
	joy_fd=open(JOY_DEV,O_NONBLOCK); // use non-blocking mode
	
	// Get the name of the joystick
    ioctl(joy_fd, JSIOCGNAME(sizeof joystick_name), joystick_name);
    
    // Get the number of axes on the joystick
    ioctl(joy_fd, JSIOCGAXES, &NumOfAxix);

    // Get the number of buttons on the joystick
    ioctl(joy_fd, JSIOCGBUTTONS, &NumOfButton);	
	
	ButtonState=0;
	AxisState=0;
}

void eventRead()
{
    for (int i=0;i<64;i++)
    {
        value[i]=99;
        axisbuttonNumber[i]=99;
    }
    LoopCount=0;
	while(read(joy_fd,&js,sizeof(js))>0)
	{
        
		switch (js.type & ~JS_EVENT_INIT)
		{
			case JS_EVENT_BUTTON:
            {if (js.value)
					ButtonState |= (1<<js.number);
				else
					ButtonState &= ~(1<<js.number);
				value[LoopCount]=js.value;
			    axisbuttonNumber[LoopCount]=js.number;
                LoopCount++;
				break;}
            case JS_EVENT_AXIS:
            {if (js.value>0)
                  AxisState |= (1<<2*js.number);
               else if (js.value<0)
                  AxisState |= (1<<(2*js.number+1));
			   else
			   {AxisState &= ~(1<<(2*js.number+1));
			   AxisState &= ~(1<<2*js.number);}
			  value[LoopCount]=js.value;
			  axisbuttonNumber[LoopCount]=js.number;
              LoopCount++;
              break;}
		}
	};
}

int returnJSInfo(int16_t * valueReturn, int16_t * numberReturn)
{
	for (int i=0;i<LoopCount+1;i++)
    {
        valueReturn[i]=value[i];
        numberReturn[i]=axisbuttonNumber[i];
    }
    return LoopCount;
}


unsigned char returnButtonState()
{
    // only return the lateset state
	return ButtonState;
}

unsigned char returnAxisState()
{
    // only return the lateset state
	return AxisState;
}