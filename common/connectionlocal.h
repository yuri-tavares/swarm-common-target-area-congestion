/*
  This class encapsulates information about message sending, 
  server/client function to enable communication among robots.
*/

#include <deque>
#include <math.h>
#include <vector>
#include "FinalLog.h"

using namespace std;

#ifndef _CONNECTION_LOCAL_H_
#define _CONNECTION_LOCAL_H_


//Area (in meters) of robot's communication radius
#define commArea 3

//SIZE of messages
#define SIZE_MSG 9

//Index for each field of message vector
#define MSG_POS_TYPE       0 //Type of message
#define MSG_POS_MY_X       1 //X coordinate of robot position
#define MSG_POS_MY_Y       2 //Y coordinate of robot position
#define MSG_POS_WAYPOINT_X 3 //X coordinate of robot's waypoint 
#define MSG_POS_WAYPOINT_Y 4 //Y coordinate of robot's waypoint 
#define MSG_POS_MY_ID      5 //Robot's id
#define MSG_POS_MY_VX      6 // x speed
#define MSG_POS_MY_VY      7 // y speed
#define MSG_POS_MY_ETA     8

class Message{
  public:
  double msg[SIZE_MSG];
  Message(double* a_msg){
    unsigned int i; 
    for (i=0;i<SIZE_MSG;i++) msg[i] = a_msg[i];
  }
  Message(double* a_msg,int size_msg){
    int i; 
    int size = min(size_msg,SIZE_MSG);
    for (i=0;i<size;i++) msg[i] = a_msg[i];
  }
  Message(){
    unsigned int i; 
    for (i=0;i<SIZE_MSG;i++) msg[i] = 0.0;
  }
};

typedef deque<Message> MessageContainer;
typedef vector<MessageContainer> Pool_t;


//This class encapsulates information concerning message sending handling.
//This class also log statistics using FinalLog class.
//Every robot have this class but the buffer of messages is common and static.
class ConnectionLocal{
  private:
    //Message Pool
    Pool_t* pool;

    //Id for my pool
    unsigned int pool_id;

    //number of messages sent
    unsigned int numMessages;

    //Given a message, compute the distance between the sender and the receiver
    double dist(double m_x, double m_y, double *msg);

    #ifdef MESSAGES_LOG
    //Log of messages sent
    ofstream messagesLog;
    #endif
  public:
    ConnectionLocal(Pool_t* apool);
 
    //Receive a msg. With received correctly, returns true
    bool receiveMsg(double m_x, double m_y, unsigned int m_id, double *msg, unsigned int size_msg);

    //send a msg
    void sendMsg(double *msg, unsigned int size_msg);

    //initialize the informations for connection
    void init_connection(string path);

    //informs the logger that the robot using this class end your task, refreshing statistics
    void finish(int m_id, int numIterationsEnter, int numIterationsExit, int numStalls);
    
    //informs the logger that the robot using this class end your task, refreshing statistics
    void finish(int m_id, int numIterationsEnter, int numIterationsExit, int numStalls, Stg::usec_t sim_time, Stg::usec_t reaching_goal_time, double maxVel, double minDist, Stg::usec_t leavingTime);
    
    //informs the logger that the robot using this class end your task, refreshing statistics
    void finish(int m_id, int numIterationsEnter, int numIterationsExit, int numStalls, Stg::usec_t sim_time, Stg::usec_t reaching_goal_time, double maxVel, double minDist, double meanDist, double varDist, unsigned long ndist, double meanVel, double varVel, unsigned long nveloc, Stg::usec_t leavingTime);
};

#endif
