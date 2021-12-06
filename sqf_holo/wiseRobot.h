/*
  This file implements a new coordination.
*/

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stage.hh>
#include <iostream>
#include <limits>
#include <fstream>
#include <signal.h>
#include <string>
#include <sstream>
#include "../common/connectionlocal.h"
#include "../common/commonConfig.h"
#include "../common/commonDefs.h"
#include "../common/util.h"
#ifdef DEBUG_FORCES
#include "../common/forcevisualizer.h"
#endif

using namespace std;
using namespace Stg;

class WiseRobot
{
  public:
   //Initialize all robot data (pose, connection, velocity, etc.)
   void init(int id, int numRobots, string const& name_run, string const& path, double new_waypointDist);
   
   //Finish robot. Here only display a message for looking with the simulation is still running.
   void finish();
   
   //Implements the main loop of robot. 
   //Also contain robot controller and 
   //probabilistic finite state machine codes
   //Constructor. pool is the message pool to send and receive msgs
   WiseRobot(Pool_t *pool);

   void walk();
   
   //Pointers to classes used in Stage
   ModelPosition* pos;
   ModelRanger* laser;
   World* theWorld; //ゴ ゴ ゴ ゴ
   
   //This member allows visualize forces for debug
   #ifdef DEBUG_FORCES
     ForceVisualizer fv;
   #endif
  private:
   #include "../common/commonMembers.cpp"
   #include "../common/regionMembers.cpp"

   //Alter the values of fx and fy, adding repulsion force.
   void obstaclesRepulsionForces(double &fx, double &fy);

   //Boolean for check is robot is already stalled (duh!)
   bool alreadyStalled;

   //Number of times that robot became stalled
   int stalls;

   //for logging maximum velocity and minimum distance
   static double maxVelocity, minDistance;

   static double target_dist;
   
   //Variables for calculating mean and variance of distance and velocity;
   double mean_distance, var_distance, mean_velocity, var_velocity;
   unsigned long n_distances, n_velocities;
   
   ofstream logd, logv;
};

//Function used on Stage simulation. It specifies 
//what the robot will do while walking
//See commonMethods.cpp for implementation in the folder common
int PositionUpdate(Model *pos, WiseRobot *robot);


