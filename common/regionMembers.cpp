/*
Commom members to the class WiseRobots that uses limitation in region
by two lines.
Every test with region delimited by two lines uses these functions.
If happens that some of the methods in this file is changed to
a specific test, it must to be eliminated from here and put on
specific test.

This file matchs with regionMethods.cpp

Note: I think that using some type of override, or abstract class can 
solve this. But I don't think that this project will grow so much.
*/

   //State of robot (for prob. state machine's use)
   int m_state;
   
   //Point in the map where the robot must wait, before it goes to goal
   double waitX;
   double waitY;

   //Log of robot's position
   #ifdef GENERAL_LOG
   ofstream log;
   #endif
   
   //If enabled message log, this is the message log file ofstream
   #ifdef MESSAGES_LOG
   ofstream messagesLog;
   #endif  

   //Counts the number of iterations that this robot have seen a robot in front
   unsigned int inFrontMisses;

   //Counts the number of iterations to send another message
   unsigned int messageCount;
   
   //Counts the number of iterations to do things after some cycles, like send messages after N cycles.
   unsigned int iteration;
   
   //Calculate the angle between the vector from this 
   //robot to goal and the vector from this robot and another robot
   double calcAngTarget(double m_x, double m_y, double tx, double ty);

   //calculate the distance between goal and the goal on the received message
   double distWaypoint(double *msg);

