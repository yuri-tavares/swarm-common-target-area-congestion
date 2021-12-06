/*
Implements a robot that coordinates with others to enter into goal region
through a enter region and exit from goal using a exit region.
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

//Returns true if a angle (in radians) is in permitted range to aply
//repulsion forces.
//Range is -PI to PI.
bool WiseRobot::verifyAngle(double radians){
  //Coloca o argumento entre -PI e PI
  radians = fmod(radians,2*PI);
  if (radians < -PI){
    radians += 2*PI;
  }
  else if (PI < radians){
    radians -= 2*PI;
  }
  /*
    Se estiver na Região deve-se verificar os quatro casos.
    Estes quatro casos dizem respeito as quatro possibilidades
    das duas retas cortarem o círculo trigonométrico.
    Reta no sentido da barra (/) pode corta o círculo considerando a parte acima
    da reta (i) ou abaixo (ii) e a reta no sentido da contra-barra (\) cortando o círculo,
    considerando a parte de cima (iii) ou de baixo (iv).
  */
  if (inRegion_X_o(m_x,m_y) && pho() < DISTANT_RADIUS){
    const double ALPHA_RAD = (PI/180.0)*ALFA; //valor de ALFA em rads
    bool condition1 = ((PI/2 /*90 graus*/ - ALPHA_RAD/2 <= radians) && (radians <= PI /*180 graus*/)) ||
                      ((-PI /*180 graus*/ <= radians) && (radians <= -PI/2 /*-90 graus*/ - ALPHA_RAD/2));
    bool condition2 = ((0 >= radians) && (radians >= -PI/2 /*-90 graus*/ + ALPHA_RAD/2)) ||
                      ((0 <= radians) && (radians <= PI/2 /*90 graus*/ + ALPHA_RAD/2));
    if (m_x > destineX && m_y > destineY){
      //caso (i)
      return condition1;
    }
    else if (m_x <= destineX && m_y > destineY){
      //caso (iii)
      return condition2;
    }
    else if (m_x <= destineX && m_y <= destineY){
      //caso (ii)
      return !condition1;
    }
    else /*if (m_x > destineX && m_y <= destineY)*/{
      //caso (iv)
      return !condition2;
    }
  }
  //caso esteja dentro da região de saída diminui a força repulsiva
  else if ((currentWaypoint != 0) && //<--- esta condição simula que está no estado SAINDO
           (distance(m_x,m_y,waypoints[0][0],waypoints[0][1]) <= DISTANT_RADIUS)) {
    return true;
  }
  //caso esteja dentro da região
  else if (pho() <= okDist){
    return true;
  }
  else if (!inRegion_X_o(m_x,m_y)){
    return true;
  }
  else{
    return false;
  }

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

        _fx = (verifyAngle(m_th + getBearing(i)))?_fx/2:_fx;
        _fy = (verifyAngle(m_th + getBearing(i)))?_fy/2:_fy;
      
        fx_ += _fx;
        fy_ += _fy;
      
        fx += _fx;
        fy += _fy;

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
  double angTarget;
  double linAccel;
  double rotAccel;
  
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
    connection.finish(m_id,numIterationsReachGoal,numIterations,stalls,sim_time,reachingTargetTime,maxVelocity,minDistance);
  }

  if ((testTime - FINISH_TIME < sim_time) && !finishedBySimTime )
  {
    finishedBySimTime = true;
    connection.finish(m_id,numIterationsReachGoal,numIterations,stalls,sim_time,sim_time,maxVelocity,minDistance);
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


  // Robots in the exit region must go to the entry region
  if (!(inRegion_X(m_x,m_y)) && pho() > dangerDist && (pho() <= DISTANT_RADIUS) && currentWaypoint == 0)
  {
     goToRegion_X();
     fx = (waitX - m_x);
     fy = (waitY - m_y);

     norm = sqrt(pow(fx,2) + pow(fy,2));

     fx = Ka*fx/norm;
     fy = Ka*fy/norm;
  }
  else
  {
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

        if (abs(fx) > 0.1 || abs(fy) > 0.1)
        {
           norm = sqrt(pow(fx,2) + pow(fy,2));

           fx = 0.1*Ka*fx/norm;
           fy = 0.1*Ka*fy/norm;
        }
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

  if (fx == 0 && fy == 0) 
      angTarget = m_th;
  else  
      angTarget = atan2(fy, fx);

  linAccel = Kl*(fx*cos(m_th) + fy*sin(m_th));
  rotAccel = Kr*(angDiff(angTarget, m_th));

  linAccel += -Kdp*linSpeed;
  rotAccel += -Kdp*rotSpeed;

  linSpeed = linSpeed + linAccel*(TIME_STEP);
  rotSpeed = rotSpeed + rotAccel*(TIME_STEP);

  this->pos->SetXSpeed(linSpeed);
  this->pos->SetTurnSpeed(rotSpeed);

  Velocity veloc = this->pos->GetVelocity();
  if (veloc.x > maxVelocity) 
    maxVelocity = veloc.x; 

  #ifdef GENERAL_LOG
  log << m_x << " " << m_y << " " 
      << destineX << " " << destineY << " "
      << waitX << " "<< waitY << " "
      << fx << " "<<  fy << " "
      << linSpeed << " "<<  rotSpeed << " "
      << inFrontMisses << " " << m_state << " "
      << linAccel << " " << rotAccel << " "
      << endl;
  #endif
}

//Returns true if a robot is inside region formed by two lines 
//(forming a X)
// AKA THE ENTRY REGION
bool WiseRobot::inRegion_X(double x, double y){
  if (ALFA == 0 ) return false;
  const double alpha = (PI/180.0)*(90 - ALFA/2);
  const double gama =  (PI/180.0)*(90 + ALFA/2);
  const double deltaY = y - destineY;
  const double deltaX = x - destineX;
  if (y > destineY){
    // check that deltaY is higher than the upper right line 
    // and the upper left line
    bool ret = (deltaY - tan(alpha)*deltaX >= 0) && (deltaY - tan(gama)*deltaX >= 0);
    return ret;
  }
  else{
    // check that deltaY is lower than the lower right line 
    // and the lower left line
    bool ret = (deltaY - tan(alpha)*deltaX <= 0) && (deltaY - tan(gama)*deltaX <= 0);
    return ret;
  }  
}

//Returns true if a robot is inside region formed by two lines 
//(forming a X) and outside danger region, given (x,y) as center of region.
// CHECK THAT position x,y is not in the danger region of the entry region.
bool WiseRobot::inRegion_X_o(double x, double y){
  bool circ = (pho() > dangerDist);
  return circ && inRegion_X(x,y);
}

/*
Alter the values of waitX  and waitY so that
the robot goes to a point next from region forming a X
*/
void WiseRobot::goToRegion_X(){
  const double alpha = (PI/180.0)*(90 - ALFA/2);
  const double gama =  (PI/180.0)*(90 + ALFA/2);
  const double deltaX = m_x - destineX;
  const double deltaY = m_y - destineY;
  #if !Y_MAIS_PROXIMO
    const double tanalpha = tan(alpha);
    const double tangama = tan(gama);
  #endif
  if ( ((m_y > destineY) && (m_x > destineX)) ||
   ((m_y <= destineY) && (m_x <= destineX))    )
  #if Y_MAIS_PROXIMO
  /*Os robos irão para o y mais proximo*/
  /*INICIO*/
  {
    waitX = m_x;
    waitY = destineY + tan(alpha)*deltaX;
  }
  else{
    waitX = m_x;
    waitY = destineY + tan(gama)*deltaX;    
  }
  /*FIM Os robos irão para o y mais proximo*/
  #else
  /*Os robos vao para o mais próximo da reta*/
  /*INICIO*/
  {
    const double d = (deltaY - tanalpha*(deltaX) )/sqrt(tanalpha*tanalpha + 1);
    const double beta = alpha - (PI/2);
    waitX = m_x + d * cos(beta);
    waitY = destineY + tanalpha*( waitX-destineX);
  }
  else{
    const double d = (deltaY - tangama*( deltaX))/sqrt(tangama*tangama + 1);
    const double beta = (PI/2) + gama;
    waitX = m_x + d * cos(beta);
    waitY = destineY + tangama*(waitX-destineX);    
  }
  /*FIM Os robos vao para o mais próximo da reta*/
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

