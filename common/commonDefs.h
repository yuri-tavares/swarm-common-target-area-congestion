#ifndef _COMMON_DEFS_H_
#define _COMMON_DEFS_H_

#include <math.h>

#define PI M_PI
#define TIME_STEP 0.01 // "Speed" of execution

//Define the codes to each state of prob. finite state machine
//States
#define GOING       0
#define WAIT        1 
#define I_DONT_CARE 2
#define WAIT_A_LOT  3
#define GOING_OUT   4

#define GO_IN_LINE  5

//define the colors used in simulation
//Colors                        R G B
#define GOING_COLOR       Color(1,0,0)
#define WAIT_COLOR        Color(0,1,0)
#define I_DONT_CARE_COLOR Color(0,0,1)
#define WAIT_A_LOT_COLOR  Color(0,1,1)
#define GOING_OUT_COLOR   Color(1,0.7,0)
#define END_COLOR         Color(0,0,0)

//Msg types
#define WATCH_OUT 0.0
#define STOP 1.0
#define I_DONT_CARE_MSG 2.0

//Size of the msg sended among robots
//#define SIZE_MSG 6

#endif
