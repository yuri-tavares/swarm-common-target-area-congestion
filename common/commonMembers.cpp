/*
Commom members to the class WiseRobots
Normally, these members are the same in every test.
If happens that some of the methods in this file is changed to
a specific test, it must to be eliminated from here and put on
specific test.

This file matchs with commomMethods.cpp

Note: I think that using some type of override, or abstract class can 
solve this. But I don't think that this project will grow so much.
*/

   //Initialize robot position data, i.e. angular and linear velocities, 
   //initial waypoint to search and seed to random number generator
   void init_position_data();

   //returns the distance between a robot and your goal
   double pho();

   //Initializes values for sensing with laser, depending on the world file used.
   void init_laser();

   //Used for get the angle of some laser beam on respect to the orientation of robot
   double getBearing(int i);

   //The name of a robot, used in some functions of Player/Stage
   string m_name;

   //Actual x coordinate of robot position
   double m_x;

   //Actual y coordinate of robot position
   double m_y;

   //Actual theta orientation of robot
   double m_th;
  
   //Identifier of the robot. Used in communication and for generate the name of robot
   int m_id;

   //Class that encapsulates informations about message sending and other stuffs to send and receive messages
   ConnectionLocal connection;

   //Current number of iterations.
   int numIterations;

   //number of iterations until reach the goal. The number of iterations to 
   //reach the goal until exiting a specified area from goal will be the difference
   //between this value and numIterations.
   int numIterationsReachGoal;

   // time in microseconds to reach the goal.
   Stg::usec_t reachingTargetTime;

   //x coordinate to the goal position
   double destineX;
  
   //y coordinate to the goal position
   double destineY;

   //Indicates if the robot finishes your execution
   bool finished;

   //Indicates if the robot finishes your execution by time limit
   bool finishedBySimTime;

   /***** From here the variables are added to be used in Stage simulations. ******/

   //index of the current waypoint that this robot search
   //See commomConfigs.h for know these values
   int currentWaypoint;

   //linear velocity of the robot
   double linSpeed;

   //rotation velocity of the robot
   double rotSpeed;

   //FOV and SAMPLES for laser. These values are token from Stage world file, and computed in init_laser()
   double LASER_FOV,LASER_SAMPLES;

