#include "FinalLog.h"


//alocating  statics members
unsigned int  FinalLog::num_robots, FinalLog::numFinished;
bool  FinalLog::initiated;
ofstream  FinalLog::logFile;
unsigned int FinalLog::numTotalIterationsReachGoal; // Total number of iterations
unsigned int FinalLog::numTotalIterationsExitGoal; // Total number of iterations
unsigned int FinalLog::numMaxIterationsReachGoal; //Maximum number of iterations to reach the goal
unsigned int FinalLog::numMaxIterationsExitGoal; //Maximum number of iterations to exit from goal
unsigned int FinalLog::numTotalStalls; //number of times that the robots stalled
unsigned int FinalLog::numMsgs; //number of messages
string FinalLog::path;
Stg::usec_t FinalLog::simTime; //simulation time in microseconds
double FinalLog::minDistance;  //minimum distance between robots detected by sensors for all robots
double FinalLog::maxVelocity; //maximum velocity achieved by any robot 
double FinalLog::meanDistance, FinalLog::varDistance;  //mean and variance of distance between robots detected by sensors for all robots
double FinalLog::meanVelocity, FinalLog::varVelocity; //mean and variance of velocity achieved by any robot 
unsigned long FinalLog::nDistance, FinalLog::nVelocity; //number of mean and variance of distance and velocity entered so far.
Stg::usec_t FinalLog::minReachingTargetTime, FinalLog::maxReachingTargetTime; //max and min simulation time in microseconds to reach the target between all robots
Stg::usec_t FinalLog::totalLeavingTime;  // sum for all robots of the time for leaving the target area

void FinalLog::init(string p){
  if (!initiated){
    initiated = true;
    simTime = 0;
    numTotalIterationsReachGoal = 0;
    numTotalIterationsExitGoal = 0;
    numMaxIterationsReachGoal = 0;
    numMaxIterationsExitGoal = 0;
    numMsgs = 0;
    numTotalStalls = 0;
    numFinished = 0;
    num_robots = 0;
    path = p;
    maxVelocity = 0;
    minDistance = INFLUENCE;
    minReachingTargetTime = testTime + FINISH_TIME + 1;
    maxReachingTargetTime = 0;
    nDistance = nVelocity = 0;
    varDistance = varVelocity = meanDistance = meanVelocity = 0;
    totalLeavingTime = 0;
  }
  num_robots++;
}

void FinalLog::refresh(unsigned int numIterationsReachGoal,
                    unsigned int numIterationsExitGoal,
                    unsigned int messages,
                    unsigned int numStalls){
  numMsgs += messages;
  numTotalStalls += numStalls;
  numTotalIterationsReachGoal += numIterationsReachGoal;
  numTotalIterationsExitGoal += numIterationsExitGoal;
  if (numIterationsReachGoal > numMaxIterationsReachGoal)
     numMaxIterationsReachGoal = numIterationsReachGoal;
  if (numIterationsExitGoal > numMaxIterationsExitGoal)
     numMaxIterationsExitGoal = numIterationsExitGoal;
}


void FinalLog::refresh(unsigned int numIterationsReachGoal,
                    unsigned int numIterationsExitGoal,
                    unsigned int messages,
                    unsigned int numStalls,
                    Stg::usec_t sim_time,
                    Stg::usec_t reaching_goal_time,
                    double maxVel,
                    double minDist,
                    Stg::usec_t leaving_goal_time){
  numMsgs += messages;
  numTotalStalls += numStalls;
  numTotalIterationsReachGoal += numIterationsReachGoal;
  numTotalIterationsExitGoal += numIterationsExitGoal;
  if (numIterationsReachGoal > numMaxIterationsReachGoal)
     numMaxIterationsReachGoal = numIterationsReachGoal;
  if (numIterationsExitGoal > numMaxIterationsExitGoal)
     numMaxIterationsExitGoal = numIterationsExitGoal;
  if (sim_time > simTime)
    simTime = sim_time;
  if (maxVelocity < maxVel)
    maxVelocity = maxVel;
  if (minDist < minDistance)
    minDistance = minDist;
  if (reaching_goal_time > maxReachingTargetTime)
    maxReachingTargetTime = reaching_goal_time;
  if (reaching_goal_time < minReachingTargetTime)
    minReachingTargetTime = reaching_goal_time;
  totalLeavingTime+=leaving_goal_time;
}

void FinalLog::saveLog(){
  logFile.open((path.c_str()));
  logFile << numTotalIterationsReachGoal + numTotalIterationsExitGoal << endl;
  logFile << numMaxIterationsReachGoal + numMaxIterationsExitGoal<< endl;
  logFile << numMsgs << endl;
  logFile << numTotalIterationsReachGoal << endl
          << numTotalIterationsExitGoal << endl;
  logFile << numMaxIterationsReachGoal << endl
          << numMaxIterationsExitGoal<< endl;
  logFile << numTotalStalls << endl;
  logFile << minReachingTargetTime << endl; //in microseconds
  logFile << maxReachingTargetTime << endl; //in microseconds
  logFile << simTime << endl; //in microseconds
  logFile << minDistance << endl;
  logFile << maxVelocity << endl;
  logFile << totalLeavingTime << endl;
  logFile.close();
}


void FinalLog::refresh(unsigned int numIterationsReachGoal,
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
                    unsigned long nveloc,
                    Stg::usec_t leaving_goal_time){
  numMsgs += messages;
  numTotalStalls += numStalls;
  numTotalIterationsReachGoal += numIterationsReachGoal;
  numTotalIterationsExitGoal += numIterationsExitGoal;
  if (numIterationsReachGoal > numMaxIterationsReachGoal)
     numMaxIterationsReachGoal = numIterationsReachGoal;
  if (numIterationsExitGoal > numMaxIterationsExitGoal)
     numMaxIterationsExitGoal = numIterationsExitGoal;
  if (sim_time > simTime)
    simTime = sim_time;
  if (maxVelocity < maxVel)
    maxVelocity = maxVel;
  if (minDist < minDistance)
    minDistance = minDist;
  if (reaching_goal_time > maxReachingTargetTime)
    maxReachingTargetTime = reaching_goal_time;
  if (reaching_goal_time < minReachingTargetTime)
    minReachingTargetTime = reaching_goal_time;
  varDistance = (nDistance*varDistance + ndist*varDist + ((meanDistance - meanDist)*(meanDistance - meanDist)*nDistance*ndist)/(nDistance + ndist))/(nDistance + ndist);
  meanDistance = (meanDistance*nDistance + meanDist*ndist)/(nDistance + ndist);
  nDistance+=ndist;
  varVelocity = (nVelocity*varVelocity + nveloc*varVel + ((meanVelocity - meanVel)*(meanVelocity - meanVel)*nVelocity*nveloc)/(nVelocity + nveloc))/(nVelocity + nveloc);
  meanVelocity = (meanVelocity*nVelocity + meanVel*nveloc)/(nVelocity + nveloc);
  nVelocity+=nveloc;
  totalLeavingTime+=leaving_goal_time;
}

void FinalLog::saveLogDistVeloc(){
  logFile.open((path.c_str()));
  logFile << numTotalIterationsReachGoal + numTotalIterationsExitGoal << endl;
  logFile << numMaxIterationsReachGoal + numMaxIterationsExitGoal<< endl;
  logFile << numMsgs << endl;
  logFile << numTotalIterationsReachGoal << endl
          << numTotalIterationsExitGoal << endl;
  logFile << numMaxIterationsReachGoal << endl
          << numMaxIterationsExitGoal<< endl;
  logFile << numTotalStalls << endl;
  logFile << minReachingTargetTime << endl; //in microseconds
  logFile << maxReachingTargetTime << endl; //in microseconds
  logFile << simTime << endl; //in microseconds
  logFile << minDistance << endl;
  logFile << maxVelocity << endl;
  logFile << meanDistance << endl;
  logFile << sqrt(varDistance) << endl;
  logFile << nDistance << endl;
  logFile << meanVelocity << endl;
  logFile << sqrt(varVelocity) << endl;
  logFile << nVelocity << endl;
  logFile << totalLeavingTime << endl;
  logFile.close();
}

void FinalLog::finish(){
  numFinished++;
  if (numFinished == num_robots){
    saveLog(); 
    exit(0);
  }
}

void FinalLog::finishDistVeloc(){
  numFinished++;
  if (numFinished == num_robots){
    saveLogDistVeloc(); 
    exit(0);
  }
}

