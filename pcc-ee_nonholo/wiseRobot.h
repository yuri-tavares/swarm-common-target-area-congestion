/*
Implements a robot that coordinates with others to enter into goal region
through a enter region and exit from goal using a exit region.

In this experiment, GOING and IMPATIENT robots go to enter region.
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

#include "alfa.h"

using namespace std;
using namespace Stg;

class WiseRobot
{
  public:
   //Constructor. pool is the message pool to send and receive msgs
   WiseRobot(Pool_t *pool);

   //Initialize all robot data (pose, connection, velocity, etc.)
   void init(int id, int numRobots, double prob, 
             string const& name_run, string const& path, double new_waypointDist);
   
   //Finish robot. Here only display a message for looking with the simulation is still running.
   void finish();
   
   //Implements the main loop of robot. 
   //Also contain robot controller and 
   //probabilistic finite state machine codes
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
   
   //Function to send warnings between robots. In other words, send the control messages
   void sendWarning();
   
   //Alter the values of fx and fy, adding repulsion force. GOING and IMPATIENT robots
   //have reduced repulsive forces.
   void obstaclesRepulsionForces(double &fx, double &fy);
   
   //Receive messages from robots in the proximity and changes state depending on its type
   //and relative position
   void communicate();
   
   /*
   Alter the values of waitX  and waitY so that
   the robot goes to a point next from region forming a X
   */
   void goToRegion_X();
   
   //Returns true if a robot is inside region formed by two lines 
   //(forming a X) and outside danger region, given (x,y) as center of region.
   bool inRegion_X_o(double x, double y);
   
   //Returns true if a robot is inside region formed by two lines 
   //(forming a X)
   bool inRegion_X(double x, double y);

   //Returns true if a angle (in radians) is in permitted range to aply repulsion forces
   bool verifyAngle(double radians);

   //Number of times that robot became stalled
   int stalls;

   //Boolean for check is robot is already stalled (duh!)
   bool alreadyStalled;

   //Probability of robot change from WAIT state to IMPATIENT
   static double m_prob;

   //for logging maximum velocity and minimum distance
   static double maxVelocity, minDistance;
   
   // distances from the target for PCC algorithm: okDist is Free region radius and dangerDist is Danger region radii
   static double okDist, dangerDist;

   static double target_dist;
   
};

//Function used on Stage simulation. It specifies 
//what the robot will do while walking
//See commonMethods.cpp for implementation in the folder common
int PositionUpdate(Model *pos, WiseRobot *robot);


