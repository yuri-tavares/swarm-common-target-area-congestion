/*
Implements a robot that coordinates with others to enter into goal region
through a enter region and exit from goal using a exit region.
*/

#include "wiseRobot.h"

#include "../common/regionMethods.cpp"
#include "../common/commonMethods.cpp"

double WiseRobot::dangerDist, WiseRobot::dist;
double WiseRobot::maxVelocity = 0, WiseRobot::minDistance = INFLUENCE;

//Initialize all robot data (pose, connection, velocity, etc.)
void WiseRobot::init(int id, int numRobots, double distance, string const& name_run, string const& path)
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

   dist = distance; 
   dangerDist = dist + 2.2;

   init_position_data();
   init_laser();
   finished = finishedBySimTime = false;

   stalls = 0;
   alreadyStalled = false;
}

//Constructor. pool is the message pool to send and receive msgs
WiseRobot::WiseRobot(Pool_t *pool):connection(pool){}

//Finish robot. Here only display a message for looking with the simulation is still running.
void WiseRobot::finish()
{
   //This message is here to see how many robots ended while experimentation scripts are ongoing, then I can see if it is stopped.
   cout << "Robot " << m_id << " finished!" << endl;
   #ifdef GENERAL_LOG
   log.close();
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
  else if (m_state == GOING_OUT){
    return true;
  }
  else if (!inRegion_X_o(m_x,m_y)){
    return true;
  }
  else{
    return false;
  }

}


//Alter the values of fx and fy, adding repulsion force. GOING and IMPATIENT robots
//have reduced repulsive forces.
void WiseRobot::obstaclesRepulsionForces(double &fx, double &fy)
{  
   double Kobs = Ke; // Weight of the obstacle repulsion forces
   double distance;
   double dx = 0;
   double dy = 0;
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

        double _fx = -Kobs*(1.0/distance - 1.0/influence)*(1.0/pow((double)distance,2))*(dx/distance);
        double _fy = -Kobs*(1.0/distance - 1.0/influence)*(1.0/pow((double)distance,2))*(dy/distance);

        _fx = (verifyAngle(m_th + getBearing(i)))?_fx/2:_fx;
        _fy = (verifyAngle(m_th + getBearing(i)))?_fy/2:_fy;
      
        fx_ += _fx;
        fy_ += _fy;
      
        fx += _fx;
        fy += _fy;
      }
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

  if (pho() < dist && m_state == GOING)
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

  if (m_state == GOING)
  {
      //Robô GOING deve estar na região X, mas pode estar dentro do circulo ao redor do GOAL
      if (!(inRegion_X(m_x,m_y) || pho() < dangerDist) && (pho() <= DISTANT_RADIUS)){
        goToRegion_X();
        fx = (waitX - m_x);
        fy = (waitY - m_y);
      }
      else{
        fx = (destineX - m_x);
        fy = (destineY - m_y);
      }
      
      norm = sqrt(pow(fx,2) + pow(fy,2));

      fx = Ka*fx/norm;
      fy = Ka*fy/norm;
  }
  else if (m_state == GOING_OUT)
  {
    fx = (destineX - m_x);
    fy = (destineY - m_y);
    
      
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
      << " " << m_state << " "
      << x_accel << " " << y_accel << " "
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

/*
Alter the values of waitX  and waitY so that
the robot goes to a a point outside de danger region
(simbolized by o in the method's name)
*/
void WiseRobot::goToRegion_o(){
  const double alpha = (PI/180.0)*(90 - ALFA/2);
  const double gama =  (PI/180.0)*(90 + ALFA/2);
  const double deltaX = m_x - destineX;
  const double deltaY = m_y - destineY;
  double r = dangerDist + 0.01;
  double x = destineX + r * (deltaX/hypot(deltaX,deltaY));
  double y = destineY + r * (deltaY/hypot(deltaX,deltaY));
  if (inRegion_X(x,y)){
    waitX = x;
    waitY = y;
  }
  else{
    waitX = x;
    if ( ((y > destineY) && (x > destineX)) ||
	  ((y <= destineY) && (x <= destineX)) )
    {
      waitY = destineY + tan(alpha)*(x-destineX);
    }
    else
    {
      waitY = destineY + tan(gama)*(x-destineX);
    }
  }
}

/*
Alter the values of waitX and waitY so that 
the robot goes to a point of the region
next to it. This point can be a point on lines forming X
or a point outside de danger region (simbolized 
by o in the method's name).
*/
void WiseRobot::goToRegion_X_o(){
  /* Se os robôs estão longe da região próxima do alvo, 
     eles vão para o ponto mais próximo da reta.
     Caso contrário, vai para a borda mais próxima da região circular e 
     das retas.
  */
  if (pho() > dangerDist){
    goToRegion_X();
  }
  else{  
    goToRegion_o();
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

   robot->init(atoi(tokens[1].c_str()), atoi(tokens[2].c_str()), atof(tokens[3].c_str()), tokens[4], tokens[5]);
   #ifdef DEBUG_FORCES
   robot->pos->AddVisualizer( &robot->fv, true );
   #endif
   return 0;
}

