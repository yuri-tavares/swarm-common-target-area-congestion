#ifndef _COMMONDEFS_H_
#define _COMMONDEFS_H_


//Define the codes to each state of prob. finite state machine
//States
typedef enum _States {
  GOING, 
  GOING_OUT, 
  ENTERING_OUTSIDE,
  ENTERING_LINE1,
  ENTERING_CURVE,
  LEAVING_CURVE,
  LEAVING_LINE2,
} States;

//define the colors used in simulation
//Colors                              R G B
#define GOING_COLOR             Color(1,0,0)         //red
#define ENTERING_OUTSIDE_COLOR  Color(0,1,1)         //cyan
#define ENTERING_LINE1_COLOR    Color(0,0,1)         //blue
#define ENTERING_CURVE_COLOR    Color(1,0,1)         //magenta
#define LEAVING_CURVE_COLOR     Color(1,1,0)         //yellow
#define LEAVING_LINE2_COLOR     Color(1,165./255.,0) //orange
#define END_COLOR               Color(0,0,0)         //black

#endif
