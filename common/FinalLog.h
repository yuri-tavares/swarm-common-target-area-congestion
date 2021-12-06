#include <fstream>
#include <string>
#include <stage.hh>
#include <stdlib.h>
#include <sstream>
#include "commonConfig.h"

using namespace std;

class FinalLog{
  private:
    static unsigned int num_robots,numFinished;
    static bool initiated;
    static ofstream logFile;
    static unsigned int numTotalIterationsReachGoal; // Total number of iterations
    static unsigned int numTotalIterationsExitGoal; // Total number of iterations
    static unsigned int numMaxIterationsReachGoal; //Maximum number of iterations to reach the goal
    static unsigned int numMaxIterationsExitGoal; //Maximum number of iterations to exit from goal
    static unsigned int numTotalStalls; //number of times that the robots stalled
    static unsigned int numMsgs; //number of messages
    static Stg::usec_t simTime; //simulation time in microseconds
    static Stg::usec_t minReachingTargetTime, maxReachingTargetTime; //max and min simulation time in microseconds to reach the target between all robots
    static string path;
    static double minDistance; //minimum distance between robots detected by sensors for all robots
    static double maxVelocity; //maximum velocity achieved by any robot 
    static double meanDistance, varDistance;  //mean and variance of distance between robots detected by sensors for all robots
    static double meanVelocity, varVelocity; //mean and variance of velocity achieved by any robot 
    static unsigned long nDistance, nVelocity; //number of mean and variance of distance and velocity entered so far.
    
    static void saveLog();    

    static void saveLogDistVeloc();

  public:
    static void init(string path);
    
    static void refresh(unsigned int numIterationsReachGoal,
                        unsigned int numIterationsExitGoal,
                        unsigned int messages,
                        unsigned int numStalls);
                        
    static void refresh(unsigned int numIterationsReachGoal,
                        unsigned int numIterationsExitGoal,
                        unsigned int messages,
                        unsigned int numStalls,
                        Stg::usec_t sim_time,
                        Stg::usec_t reaching_goal_time,
                        double maxVel,
                        double minDist);
    
    static void refresh(unsigned int numIterationsReachGoal,
                        unsigned int numIterationsExitGoal,
                        unsigned int messages,
                        unsigned int numStalls,
                        Stg::usec_t sim_time,
                        Stg::usec_t reaching_goal_time,
                        double maxVel,
                        double minDist,
                        double meanDist,
                        double varDist,
                        unsigned long ndist,
                        double meanVel, 
                        double varVel,
                        unsigned long nveloc);

    static void finishDistVeloc();

    static void finish();
};
