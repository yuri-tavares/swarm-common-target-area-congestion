/*
  This file implements a robot without coordination.
  The robot moves to the target using potential field and repels from another robot
  using this method.
*/

#include "wiseRobot.h"

#include "../common/regionMethods.cpp"
#include "../common/commonMethods.cpp"

double WiseRobot::target_dist;
double WiseRobot::maxVelocity = 0, WiseRobot::minDistance = INFLUENCE;

//Constructor. pool is the message pool to send and receive msgs
WiseRobot::WiseRobot(Pool_t *pool):connection(pool){}

//Initialize all robot data (pose, connection, velocity, etc.)
void WiseRobot::init(int id, int numRobots, string const& name_run, string const& path, double new_waypointDist)
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

   target_dist = new_waypointDist;
   init_position_data();
   init_laser();
   finished = finishedBySimTime = false;
   stalls = 0;
   alreadyStalled = false;
   
   mean_distance = INFLUENCE; var_distance = 0; mean_velocity = 0; var_velocity = 0;
   n_distances = n_velocities = 0;
   #ifdef LOG_VEL_DIST
     logv.open((m_name+"_v").c_str());
     logd.open((m_name+"_d").c_str());
   #endif
}

//Finish robot. Here only display a message for looking with the simulation is still running.
void WiseRobot::finish()
{
   //This message is to see how many robots ended while experimentation scripts are ongoing, then I can see if it is stopped.
   cout << "Robot " << m_id << " finished!" << endl;
   #ifdef LOG_VEL_DIST
     logv.close();
     logd.close();
   #endif
   #ifdef GENERAL_LOG
     log.close();
   #endif
}

//Alter the values of fx and fy, adding repulsion force.
void WiseRobot::obstaclesRepulsionForces(double &fx, double &fy)
{  
   double Kobs = Ke; // Weight of the obstacle repulsion forces
   double distance;
   double dx = 0;
   double dy = 0;
   double fx_ = 0 ,fy_ = 0;

   const std::vector<meters_t>& scan = laser->GetSensors()[0].ranges;
   uint32_t sample_count = scan.size();
   
   double influence =  INFLUENCE;
   const double Imin = 1.;
   double min_scan = influence;
   if (m_state == GOING || 
       (m_state == GOING_OUT && 
        finished))
   {
     influence = Imin;
   }
   else if ((m_state == WAIT) && (m_y > destineY) && (fabs(m_x - destineX) < INFLUENCE - Imin))
   {
     influence = Imin + fabs(m_x - destineX);
   }
   for(uint32_t i = 0; i < sample_count; i++)
   {
      distance = scan[i];
      min_scan = min(distance, min_scan);
      if (distance <= influence)
      {
         if (distance <= minDistance && !finished){
           minDistance = distance;
         }
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
      }
   }
   if (!finished){
     n_distances++;
     IterativeMeanVariance(min_scan, n_distances, &mean_distance, &var_distance);
     #ifdef LOG_VEL_DIST
       logd << min_scan << endl;
     #endif
   }
   #ifdef DEBUG_FORCES
   fv.setRepulsiveForces(fx_,fy_);
   #endif
}

//Implements the main loop of robot. 
//Also contain robot controller and 
//probabilistic finite state machine codes
void WiseRobot::walk(){
  double fx = 0;
  double fy = 0;
  double norm = 0;
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

  if ((pho() < target_dist) & !finished)
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
    connection.finish(m_id,numIterationsReachGoal,numIterations,stalls,sim_time,reachingTargetTime,maxVelocity,minDistance,mean_distance, var_distance, n_distances, mean_velocity, var_velocity, n_velocities, sim_time-reachingTargetTime);
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

  
  if (m_state != GOING_OUT)
  {
    if (distance(m_x, m_y, waypoints[0][0], waypoints[0][1]) < DISTANT_RADIUS && (m_y < destineY || abs(m_x - destineX) > target_dist))
    {
      m_state = WAIT;
      pos->SetColor(WAIT_COLOR);
      // enter in rotation !
      if (m_x - destineX > 0)
      {
        fx = - (m_y - destineY);
        fy =   (m_x - destineX);
      }
      else
      {
        fx =   (m_y - destineY);
        fy = - (m_x - destineX);
      }
      
      norm = sqrt(pow(fx,2) + pow(fy,2));
      fx = Ka*fx/norm;
      fy = Ka*fy/norm;
    }
    else
    {
      m_state = GOING;
      pos->SetColor(GOING_COLOR);
      // go to target
      fx = (destineX - m_x);
      fy = (destineY - m_y);
      
      norm = sqrt(pow(fx,2) + pow(fy,2));
      fx = Ka*fx/norm;
      fy = Ka*fy/norm;
    }
  
  }
  else // m_state == GOING_OUT
  {
    if (distance(m_x, m_y, waypoints[0][0], waypoints[0][1]) < DISTANT_RADIUS)
    {
      // enter in rotation !
      if (destineX > 0)
      {
        fx = - (m_y - waypoints[0][0]);
        fy =   (m_x - (waypoints[0][1] + DISTANT_RADIUS));
      }
      else
      {
        fx =   (m_y - destineY);
        fy = - (m_x - (waypoints[0][1] - DISTANT_RADIUS));
      
      }
    }
    else
    {
      // go to target
      fx = (destineX - m_x);
      fy = (destineY - m_y);
    }
    
    norm = sqrt(pow(fx,2) + pow(fy,2));
    fx = Ka*fx/norm;
    fy = Ka*fy/norm;
  }

  #ifdef DEBUG_FORCES
  fv.setAttractiveForces(fx,fy);
  #endif

  obstaclesRepulsionForces(fx, fy);

  saturation(fx, fy, maxForce);
  #ifdef DEBUG_FORCES
  fv.setResultantForces(fx,fy);
  #endif

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
  if (!finished){
    Velocity veloc = this->pos->GetVelocity();
    double linvec = hypot(veloc.x,veloc.y);
    if (linvec > maxVelocity) 
      maxVelocity = linvec; 
    n_velocities++;
    IterativeMeanVariance(linvec, n_velocities, &mean_velocity, &var_velocity);
    #ifdef LOG_VEL_DIST
      logv << linvec << endl;
    #endif
  }
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

   robot->init(atoi(tokens[1].c_str()), atoi(tokens[2].c_str()), tokens[3], tokens[4], atof(tokens[5].c_str()));
   #ifdef DEBUG_FORCES
   robot->pos->AddVisualizer( &robot->fv, true );
   #endif
   return 0;
}

