/*
  This file implements a robot without coordination.
*/
#ifndef _ROBOT_H_
#define _ROBOT_H_


#include <stdlib.h>
#include <stage.hh>
#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <iomanip>
#include "FinalLog.h"
#include "ConfigFile.h"
#include "commonConfig.h"
#include "commonDefs.h"
#include "util.h"
#include "option.hh"
#ifdef DEBUG_FORCES
#include "forcevisualizer.h"
#endif

using namespace std;
using namespace Stg;

#define PI M_PI


class Robot
{
  public:
    /*Constructor arguments:
       conf: file name with experiments variables.
    */
    Robot(string const& conf);
    
    //Initialize all robot data (pose, log file, velocity, etc.)
    void init(int id);
    
    //Finish robot. Here only display a message for looking with the simulation is still running.
    void finish();
    
    //Implements the main loop of robot. 
    //Also contain robot controller and 
    //probabilistic finite state machine codes
    void mainLoop();
    
    //Pointers to classes used in Stage
    ModelPosition* pos;
    ModelRanger* laser;
    World* theWorld; //ゴ ゴ ゴ ゴ
    
    //This member allows visualize forces for debug
    #ifdef DEBUG_FORCES
      ForceVisualizer fv;
    #endif
  private:
    //save data of this robot on own log
    void saveMyLog();
    
    //returns the distance between a robot and your goal
    double pho();
    
    //Subtract two angle values (in radians). The result value lies between -2 PI and 2 PI. 
    double angDiff(double end, double begin);
    
    //Initializes values for sensing with laser, depending on the world file used.
    void init_laser();
    
    //Reads a config file and calculate the static parameters.
    static void readConfigFile(string const& confFileName);
    
    // Set fx and fy to the attractive force to point (gotoX,gotoY).
    // The modulo of (fx,fy) will be Ka.
    void setAttractiveForce(double gotoX, double gotoY);
    
    // Add to fx and fy the attractive force to point (gotoX,gotoY).
    // The modulo of the force after the addition will be Ka.
    void addAttractiveForce(double gotoX, double gotoY);
    
    //Alter the values of fx and fy, adding repulsion force.
    void obstaclesRepulsionForces();
    
    //Alter the values of fx and fy, adding repulsion force by target region.
    void targetRegionRepulsionForce();

    // Set linear and turning speeds based on the force vector.
    // The linear velocity is set to a maximum value.
    void setSpeeds();
    
    //Saturate a vector to a limit modulo, keeping scale.
    void saturation(double &x, double &y, double limit);
    
    /*
    Set line 1 parameters just once.
    Argument:
      lineangle: entering ray angle (in rad and in relation to global x-axis) of the central angle region angle.
    */
    inline void setLine1PathParameters(double lineangle);

    /*
    Set line 2 parameters just once.
    Argument:
      lineangle: entering ray angle (in rad and in relation to global x-axis) of the central angle region angle.
    */
    inline void setLine2PathParameters(double lineangle);
    
    
    /* 
    Line path vector field constructor. Sets fx and fy. Let w1 be the line starting point coordinates and w2 the line ending point coordinates in the arguments explanations below.
    Arguments:
      w2_w1x_, w2_w1y_, w2_w1mod2_: x and y coordinates of the vector w2 - w1 and its modulus squared;
      w1x_, w1y_: x and y coordinates of the vector w1.
    Return true if the robot arrived next to the ending waypoint w2.
    */
    bool linePathFollowing(double w2_w1x_, double w2_w1y_, double w2_w1mod2_, double w1x_, double w1y_);
    
    /*
    Set first circular path parameters just once.
    */
    inline void setCircularPathParameters();
    
    /*
    Set circular parameters for second circular path just once.
    */
    inline void setCircularPathParameters2(int linenumber);
    
    /*
    Cicular path vector field constructor. Sets fx and fy. 
    Returns a float value: <= 0 and >= 1 indicates the robot already reached the final waypoint.
    */
    double circularPathFolowing();

    /* Follow the line-circle-line vector field.
    Reference: Vector field path following for small unmanned air vehicles.
    Conference Paper in Proceedings of the American Control Conference · July 2006
    DOI: 10.1109/ACC.2006.1657648 */
    void coolPathFollowing();
    
    //Used for get the angle of some laser beam on respect to the orientation of robot
    inline double getBearing(int i);
    
    //Check if the config file was already read.
    static bool alreadyReadConfig;
    
    //maximum linear velocity of the robot
    static double maxLinSpeed;
    
    //minimum distance between robots in corridor and minimum influence for repulsive force.
    static double d;
    
    //maximum distance from the target to the algorithm takes effect.
    static double D;
    
    //number of lanes
    static int K_path;
    
    //trajectory parameters
    static double alpha, r, dcurve, corridorLength, powtauk;
    
    //for logging maximum velocity and minimum distance
    static double maxVelocity, minDistance;
    
    /*Parameters for line trajectory 1. The line begins at w1 = (w1x,w1y) and ends at w2 = (w2x,w2y). 
    w2_w1x, w2_w1y, w2_w1mod: x and y coordinates of the vector w2 - w1 and its squared modulus;
    w1x, w1y: x and y coordinates of the vector w1.*/
    double w2_w1x, w2_w1y, w2_w1mod2, w1x, w1y;
    
    // Waypoint used on line trajectory 1 and circular trajectory 2.
    double w2x,w2y;
    
    //Centre of the circular trajectory
    double cx,cy;
    
    //starting point of the line trajectory 2.
    double w3x,w3y;

    // w1 - c's x and y coordinates. 
    double w1cx,w1cy;
    
    // waypoint in the intersection of the curved trajectory and the target region
    double wcx, wcy;
    
    //radius used on the circular trajectory vector field and value for circular path calculation.
    double r1, powrk;

    /*Parameters for line trajectory 2. The line begins at w3 = (w3x,w3y) and ends at w4 = (w4x,w2y). 
    w4_w3x, w4_w3y, w4_w3mod: x and y coordinates of the vector w4 - w3 and its squared modulus. */
    double w4_w3x,w4_w3y,w4_w3mod2;
    
    //For avoiding recalculation of the line 1, around target cicular path, towards target region path and line 2 parameters. They are false if the robot already calculate the respective path parameters.
    bool line1parametersWasNotCalculated, circularParametersWasNotCalculated, circularParameters2WasNotCalculated,toTargetRegionParametersWasNotCalculated, line2parametersWasNotCalculated;
    
    // linear and turning speed for non-holonomic and x- and y-speed for holonomic.
    double linSpeed, rotSpeed;
    
    // force vector for robot movement
    double fx,fy;
    
    //Radius of target area
    static double waypointDist;
    
    //Folder where the logs will be saved
    static string folder;
    
    //Log file name
    static string log_name;
    
    //Maximum time in miliseconds to test
    static unsigned long long testTime;
    
    // True if user wants to save the velocities and positions of the robots over the time in a log.
    static bool savePosVel;
    
    //The name of a robot, used in some functions of Player/Stage
    string m_name;
    
    //Actual x coordinate of robot position
    double m_x;
    
    //Actual y coordinate of robot position
    double m_y;
    
    //Actual theta orientation of robot
    double m_th;
   
    // Resolution of execution for discrete movement controler
    const double TIME_STEP = 0.01;
    
    //Identifier of the robot. Used in communication and for generate the name of robot
    int m_id;
    
    //State of robot in the experiment
    States m_state;
    
    //Actual number of iterations.
    int numIterations;
    
    //number of iterations until reach the goal. The number of iterations from 
    //reach the goal until exit a specified area from goal will be the difference
    //between this value and numIterations
    int numIterationsReachGoal;
    
    //x and y coordinates to the goal position
    double destinationX, destinationY;
    
    // waypoints vector index where the robot are going
    unsigned int currentWaypoint;
    
    //Indicates if the robot finished your execution
    bool finished;
    
    //Indicates if the robot finishes your execution by time limit
    bool finishedBySimTime;

    //Indicates if the robot exited from the target area
    bool exitedFromTargetRegion;
    
    //Used for changing the colour at the ending of the experiment
    int alreadyChanged;
    
    //FOV and SAMPLES for laser. These values are token from Stage world file and computed in init_laser()
    double LASER_FOV,LASER_SAMPLES;
    
    //check if robot is already stalled
    bool alreadyStalled;
    
    //Number of times that robot became stalled
    int stalls;
    
    // Used for control holonomic or non-holonomic robots.
    static bool isHolonomic;
    
    // time in microseconds to reach the goal.
    Stg::usec_t reachingTargetTime;
    
    //Variables for calculating mean and variance of distance and velocity;
    double mean_distance, var_distance, mean_velocity, var_velocity;
    unsigned long n_distances, n_velocities;
    
    ofstream log, logd, logv;
    
};

//Function used on Stage simulation. It specifies 
//what the robot will do while walking
int PositionUpdate(Model *pos, Robot *robot);


#endif
