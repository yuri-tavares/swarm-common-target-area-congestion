#ifndef _COMMON_CONFIG_H_
#define _COMMON_CONFIG_H_

#define INFLUENCE 3.0 // Normal radius of influence of the obstacles
#define LATERAL_DISTANCE 3 //Distance to consider that the robot is laterally next to the target region.
#define MEDIAL_DISTANCE 1. //Distance to consider that the robot is next from above or below of the target region.
#define DISTANT_RADIUS 13.0 //maximum radius after that robot reaches goal
#define PROB_CYCLES 40 //number of cycles test changing state probability 
#define MAX_MISSES 30 //number of checks (or cycles) that there is not any robot in front
//Waypoints chosen randomly
const int waypoints[3][2] = { {100,100}, {-999999, 100}, {999999,100}/*, {0,-999999}, {0,999999}*/ };
//const int waypoints[5][2] = { {0,0}, {-999999, 0}, {999999,0}/*, {0,-999999}, {0,999999}*/ };
#define NUMBER_OF_WAYPOINTS 2 //number of waypoints used in all tests
#define Y_MAIS_PROXIMO 0 //Esta variável define se o robô irá para o y da reta mais próximo de sua posição
//~ #define DEAD_ITERATIONS 100000 //Maximum iterations for a experiment
//~ #define CHECK_DEAD_ROBOTS //if enable, consider the above maximum number of iteration
#define MSG_CYCLES 25 //number of cycles to wait until send next message
#define Ke 0.5   //constant for repulsion forces of WAIT and LOCKED robots
#define Ki 0.5  //constant for repulsion forces of IMPATIENT and GOING robots
//#define SERVER_FINISH_OUTPUT //if defined, server outputs messages about finished robots
//#define GENERAL_LOG  //if defined, robots output log messages to stout
//#define DEBUG_FORCES //If defined, it allow compilation of force visualization codes

const double Ka = 2.5; //constant for robot controller, attraction to target
const double Kr = 3; //constant for robot controller, m/s^2/rad acceleration in rotation
const double Kl = 1; //constant for robot controller. Factor of the input acceleration (should be set to 1 by default)
const double Kdp = 10; // Weight of the damping force
const double waypointDist = 3.; //minimum distance to consider that a robot arrive to goal
const double maxForce = 2.5; //Maximum modulo for resultant force, used on robot controller
const double limit = 0.1; // To avoid a division per zero, the minimum ammount that will be allowed
const double sameWaypointOffset = 0.5; //maximum distance to consider that robots have common goal
const double congestionOkDist = waypointDist + 0.7; //minor distance to goal
const double congestionDangerDist = congestionOkDist + 1.5; //distance between goal and robots for obey protocol
//#define MESSAGES_LOG //If defined, logs are generated

const unsigned long long FINISH_TIME = 100000; //Extra time to finish simulation in microseconds;
const unsigned long long testTime = 60l*60*1000000; // 60 min converted to microseconds

#endif
