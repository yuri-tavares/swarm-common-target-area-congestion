/*
  Implements a robot that coordinates with other using a
  circular region.
  Read Marcolino & Chaimowicz 2009 ICRA paper for more information
*/

#include "wiseRobot.h"

#include "../common/regionMethods.cpp"
#include "../common/commonMethods.cpp"

double WiseRobot::okDist, WiseRobot::dangerDist, WiseRobot::target_dist, WiseRobot::m_prob;
double WiseRobot::maxVelocity = 0, WiseRobot::minDistance = INFLUENCE;

//Function to send warnings between robots. In other words, send the control messages
void WiseRobot::sendWarning()
{
   double msg[SIZE_MSG];

   if (m_state != I_DONT_CARE && pho() > okDist)
   {
      if (m_state == GOING)
        msg[MSG_POS_TYPE] = WATCH_OUT;
      else if (m_state == WAIT || m_state == WAIT_A_LOT)
        msg[MSG_POS_TYPE] = STOP;

      msg[MSG_POS_WAYPOINT_X] = destineX;
      msg[MSG_POS_WAYPOINT_Y] = destineY;
      msg[MSG_POS_MY_X] = m_x; 
      msg[MSG_POS_MY_Y] = m_y;
      msg[MSG_POS_MY_ID]= (double) m_id;
      
      if ((msg[MSG_POS_TYPE] == WATCH_OUT && pho() < dangerDist) || msg[MSG_POS_TYPE] == STOP) // I will send a WATCH_OUT only if I am near my waypoint
      {
         connection.sendMsg(msg,SIZE_MSG);
         #ifdef MESSAGES_LOG
         for (int i = 0; i < SIZE_MSG; i++){
           messagesLog << msg[i] << " ";
         }
         messagesLog << endl;
         #endif
      }
   }
}

//Constructor. pool is the message pool to send and receive msgs
WiseRobot::WiseRobot(Pool_t *pool):connection(pool){}

//Initialize all robot data (pose, connection, velocity, etc.)
void WiseRobot::init(int id, int numRobots, double prob, 
                     string const& name_run, string const& path, double new_waypointDist)
{
   m_id = id;
   m_name = "robot" + intToStr(id);

   Pose pose = pos->GetPose();
   m_x = pose.x;
   m_y = pose.y;
   m_th = pose.a;

   numIterations = numIterationsReachGoal = 0;

   connection.init_connection(path);

   m_state = GOING;
   pos->SetColor(GOING_COLOR);

   #ifdef GENERAL_LOG
   log.open((name_run + "_logs/"+m_name).c_str());
   #endif

   m_prob = prob;
   target_dist = new_waypointDist;
   okDist = target_dist + 0.7;
   dangerDist = okDist + 1.5;
   init_position_data();
   init_laser();
   finished = finishedBySimTime = false;
   #ifdef MESSAGES_LOG
   messagesLog.open((name_run + "_messages/"+m_name).c_str());
   #endif

   messageCount = 0;
   inFrontMisses = 0;
   stalls = 0;
   alreadyStalled = false;
}

//Finish robot. Here only display a message for looking with the simulation is still running.
void WiseRobot::finish()
{
   //This message is here to see how many robots ended while experimentation scripts are ongoing, then I can see if it is stopped.
   cout << "Robot " << m_id << " finished!" << endl;
   #ifdef GENERAL_LOG
   log.close();
   #endif
   #ifdef MESSAGES_LOG
   messagesLog.close();
   cout << m_name << ": message created!" << endl;
   #endif
}

//Alter the values of fx and fy, adding repulsion force.
void WiseRobot::obstaclesRepulsionForces(double &fx, double &fy)
{  
   double Kobs = Ke; // Weight of the obstacle repulsion forces
   double distance;
   double dx = 0;
   double dy = 0;
   bool found = false;
   double fx_ = 0 ,fy_ = 0;
   const std::vector<meters_t>& scan = laser->GetSensors()[0].ranges;
   uint32_t sample_count = scan.size();

   for(uint32_t i = 0; i < sample_count; i++)
   {
      distance = scan[i];
      double influence =  INFLUENCE;
      if (distance <= influence)
      {
        if (distance <= minDistance)
          minDistance = distance;
        
        dx = distance*cos(m_th + getBearing(i));
        dy = distance*sin(m_th + getBearing(i));

        if (dx < limit && dx >= 0)
           dx = limit;
        if (dx > -limit && dx < 0)
           dx = -limit;
        if (dy < limit && dy >= 0)
           dy = limit;
        if (dy > -limit && dy < 0)
           dy = -limit;

        double _fx = -Kobs*(1.0/distance - 1.0/(influence*2))*(1.0/pow((double)distance,2))*(dx/distance);
        double _fy = -Kobs*(1.0/distance - 1.0/(influence*2))*(1.0/pow((double)distance,2))*(dy/distance);

         fx += _fx;
         fy += _fy;

         fx_ += _fx;
         fy_ += _fy;

        found = true;
      }
   }
   #ifdef DEBUG_FORCES
   fv.setRepulsiveForces(fx_,fy_);
   #endif
   if (messageCount == 0)
   {
      if (found)
      {
         sendWarning();
      }
   }
   messageCount = (messageCount + 1) % MSG_CYCLES;
}

//Implements the main loop of robot. 
//Also contain robot controller and 
//probabilistic finite state machine codes
void WiseRobot::walk(){
  double fx = 0;
  double fy = 0;
  double norm = 0;
  double r;
  double x_accel;
  double y_accel;
  
  numIterations++;

  //how many times robot stall?
  if (pos->Stalled()){
    if (!alreadyStalled){
      stalls++;
      alreadyStalled = true;
    }
  }
  else{
    alreadyStalled = false;
  }

  Pose pose = pos->GetPose();
  m_x = pose.x;
  m_y = pose.y;
  m_th = pose.a;
  #ifdef DEBUG_FORCES
  fv.setPosition(m_x,m_y,m_th);
  #endif

  if (pho() < target_dist && currentWaypoint == 0)
  {
      finished = true;
      currentWaypoint = 1 + (rand() % NUMBER_OF_WAYPOINTS);
      m_state = GOING_OUT;
      pos->SetColor(GOING_OUT_COLOR);
      reachingTargetTime = theWorld->SimTimeNow();
      numIterationsReachGoal = numIterations;
      numIterations = 0;
  }

  Stg::usec_t sim_time = theWorld->SimTimeNow();
  
  if (finished && (distance(m_x,m_y,waypoints[0][0],waypoints[0][1]) >= DISTANT_RADIUS)){
    pos->SetColor(END_COLOR);
    finish();
    finished = false;
    connection.finish(m_id,numIterationsReachGoal,numIterations,stalls,sim_time,reachingTargetTime,maxVelocity,minDistance,sim_time-reachingTargetTime);
  }

  if ((testTime - FINISH_TIME < sim_time) && !finishedBySimTime )
  {
    finishedBySimTime = true;
    connection.finish(m_id,numIterationsReachGoal,numIterations,stalls,sim_time,sim_time,maxVelocity,minDistance,sim_time-reachingTargetTime);
    pos->SetColor(Color(0,0,0));
    finish();
  }

  #ifdef CHECK_DEAD_ROBOTS
  if (numIterations > DEAD_ITERATIONS)
  {
      connection.finish(m_id,numIterations,0,stalls);
      pos->SetColor(END_COLOR);
      finish();
  }
  #endif

  destineX = waypoints[currentWaypoint][0];
  destineY = waypoints[currentWaypoint][1];

  iteration = (iteration + 1) % PROB_CYCLES;

  if (m_state == WAIT  && iteration == 0)
  {
      r = (   (double)rand() / ((double)(RAND_MAX)+(double)(1)) );
      if (r < m_prob){
        m_state = I_DONT_CARE;
        pos->SetColor(I_DONT_CARE_COLOR);
      }
  }

  if (m_state == GOING || m_state == I_DONT_CARE || m_state == GOING_OUT)
  {
      fx = (destineX - m_x);
      fy = (destineY - m_y);

      norm = sqrt(pow(fx,2) + pow(fy,2));

      fx = Ka*fx/norm;
      fy = Ka*fy/norm;
  }
  else if (m_state == WAIT || m_state == WAIT_A_LOT )
  {
      fx = (waitX - m_x);
      fy = (waitY - m_y);

      if (abs(fx) > 0.01 || abs(fy) > 0.01)
      {
        norm = sqrt(pow(fx,2) + pow(fy,2));
        fx = 0.1*Ka*fx/norm;
        fy = 0.1*Ka*fy/norm;
      }
  }
  #ifdef DEBUG_FORCES
  fv.setAttractiveForces(fx,fy);
  #endif

  obstaclesRepulsionForces(fx, fy);

  saturation(fx, fy, maxForce);
  #ifdef DEBUG_FORCES
  fv.setResultantForces(fx,fy);
  #endif
  communicate();
  
  /* Control for omni robot */
  // linSpeed is repurposed to signify x_speed
  // rotSpeed is repurposed to signify y_speed
  x_accel = fx;
  y_accel = fy;
  
  x_accel += -Kdp*linSpeed;
  y_accel += -Kdp*rotSpeed;
  
  linSpeed = linSpeed + x_accel*(TIME_STEP);
  rotSpeed = rotSpeed + y_accel*(TIME_STEP);
  
  this->pos->SetSpeed(linSpeed, rotSpeed, 0);

  Velocity veloc = this->pos->GetVelocity();
  double linvec = hypot(veloc.x,veloc.y);
  if (linvec > maxVelocity) 
    maxVelocity = linvec; 

  #ifdef GENERAL_LOG
  log << m_x << " " << m_y << " " 
      << destineX << " " << destineY << " "
      << waitX << " "<< waitY << " "
      << fx << " "<<  fy << " "
      << linSpeed << " "<<  rotSpeed << " "
      << inFrontMisses << " " << m_state << " "
      << x_accel << " " << y_accel << " "
      << endl;
  #endif
}


//Receive messages from robots in the proximity and changes state depending on its type and relative position
void WiseRobot::communicate()
{
   double ang;
   bool inFront;
   inFront = false;

   double msg[SIZE_MSG];

   //aqui eh que simula a distancia da comunicacao
   bool msgReceived = connection.receiveMsg(m_x,m_y, m_id, msg, SIZE_MSG);
   
   
   if (msgReceived && distWaypoint(msg) < sameWaypointOffset && pho() > okDist )
   {
     if (msg[MSG_POS_TYPE] == WATCH_OUT)
     {
       ang = (180/3.14)*calcAngTarget(m_x, m_y, msg[MSG_POS_MY_X], msg[MSG_POS_MY_Y]);
        
       if (pho() < dangerDist && m_state == GOING && !(ang < -25 && ang > -155))
       {
         m_state = WAIT;
         pos->SetColor(WAIT_COLOR);
         waitX = m_x;
         waitY = m_y;
       }
     }
     else if (msg[MSG_POS_TYPE] == STOP)
     {
       ang = (180/3.14)*calcAngTarget(m_x, m_y, msg[MSG_POS_MY_X], msg[MSG_POS_MY_Y]);
        
       if (ang > 45 && ang < 135)
       {
         if (m_state == GOING)
         {
           m_state = WAIT_A_LOT;
           pos->SetColor(WAIT_A_LOT_COLOR);
           waitX = m_x;
           waitY = m_y;
         }
         inFront = true;
         inFrontMisses = 0;
       }
       else if (pho() < dangerDist && m_state == GOING && !(ang < -25 && ang > -155))
       {
         m_state = WAIT;
         pos->SetColor(WAIT_COLOR);
         waitX = m_x;
         waitY = m_y;
       }
     }
   }

   if (m_state == WAIT_A_LOT && !inFront){
      inFrontMisses++;
   }

   if (inFrontMisses > MAX_MISSES && (m_state == WAIT_A_LOT || m_state == WAIT))
   {
      m_state = GOING;
      pos->SetColor(GOING_COLOR);
      
      inFrontMisses = 0;
   }

   if (m_state == WAIT_A_LOT && pho() <= dangerDist){
     m_state = WAIT;
     pos->SetColor(WAIT_COLOR);
   }
}

Pool_t pool;

//Pointer to a new robot.
//Every call of this library will create a new robot
//using this pointer.
WiseRobot* robot;

extern "C" int Init(Model *mod, CtrlArgs *args){
   robot = new WiseRobot(&pool); 
   vector<string> tokens;
   Tokenize(args->worldfile, tokens); 

   ModelPosition *pmod = (ModelPosition*) mod;
   robot->pos = pmod;
   robot->theWorld = pmod->GetWorld(); //ゴ ゴ ゴ ゴ
   robot->pos->AddCallback(Model::CB_UPDATE, (model_callback_t)PositionUpdate, robot );
   robot->laser = (ModelRanger*)mod->GetChild( "ranger:1" );

   robot->laser->Subscribe(); // starts the laser updates
   robot->pos->Subscribe(); // starts the position updates

   robot->init(atoi(tokens[1].c_str()), atoi(tokens[2].c_str()), atof(tokens[3].c_str()), tokens[4], tokens[5], atof(tokens[6].c_str()));
   #ifdef DEBUG_FORCES
   robot->pos->AddVisualizer( &robot->fv, true );
   #endif
   return 0;
}

