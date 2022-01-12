#include "Robot.h"



//Initializes values for sensing with laser, depending on the world file used.
void Robot::init_laser()
{
  //init laser configuration for use in getBearing()
  ModelRanger::Sensor sensor = laser->GetSensors()[0];
  LASER_FOV  = rtod(sensor.fov);
  LASER_SAMPLES = sensor.sample_count;
  laser->vis.showArea.set(0);
}

//Function used on Stage simulation. It specifies 
//what the robot will do while walking
//See Robot.h for signature
int PositionUpdate(Model *pos, Robot *robot)
{
  robot->mainLoop();
  return 0;
}

//returns the distance between a robot and your goal
double Robot::pho()
{
   return hypot(m_x - destinationX, m_y - destinationY);
}

//Used for get the angle of some laser beam on respect to the orientation of robot
inline double Robot::getBearing(int i)
{
  return dtor(-LASER_FOV/2 + (LASER_FOV/(LASER_SAMPLES-1))*i);
}

/*Constructor arguments:
    conf: file name with experiments variables.
*/
Robot::Robot(string const& confFile)
{
  readConfigFile(confFile);
}

bool Robot::alreadyReadConfig = false;
double Robot::maxLinSpeed;
double Robot::waypointDist;
string Robot::folder;
string Robot::log_name;
unsigned long long Robot::testTime;
int Robot::K_path;
double Robot::d, Robot::D,  Robot::powtauk;
double Robot::alpha, Robot::r, Robot::dcurve, Robot::corridorLength;
double Robot::maxVelocity = 0, Robot::minDistance;
bool Robot::isHolonomic;
bool Robot::savePosVel = false;

//Reads a config file and calculate the static parameters.
void Robot::readConfigFile(string const& confFileName)
{
  if (alreadyReadConfig) return;
  ConfigFile cf(confFileName);
  try{
    maxLinSpeed  = atof(cf.valueOf("v").c_str());
    d            = atof(cf.valueOf("d").c_str());
    D            = atof(cf.valueOf("D").c_str());
    waypointDist = atof(cf.valueOf("s").c_str());
    folder       = cf.valueOf("folder");
    log_name     = cf.valueOf("log");
    K_path       = atoi(cf.valueOf("K").c_str());
    isHolonomic  = (atoi(cf.valueOf("holo").c_str()) != 0);
  }
  catch (string s) {
    cout << endl << "Robot.o: Configuration file is incorret: " << s << endl;
    exit(1);
  }
  try{
    testTime = atoll(cf.valueOf("time").c_str());
  }
  catch (string s) {
    testTime = maxTestTime;
  }
  try{
    savePosVel = (atoi(cf.valueOf("saveVelocities").c_str()) != 0);
  }
  catch (string s) {
    savePosVel = false;
  }
  alpha = 2*PI/K_path;
  r = (waypointDist * sin(alpha / 2.) - d/2.) / (1 - sin(alpha / 2.));
  dcurve = sqrt(pow(waypointDist + r,2) - pow(d/2. + r,2));
  corridorLength = D - dcurve;
  powtauk = pow(d/5,Kexpoent1);
  minDistance = d;
  alreadyReadConfig = true;
}

//Initialize all robot data (pose, log file, velocity, etc.)
void Robot::init(int id)
{
  m_id = id;
  m_name = "robot" + intToStr(id);
  Pose pose = pos->GetPose();
  m_x  = pose.x;
  m_y = pose.y;
  m_th = pose.a;
  numIterations = numIterationsReachGoal = 0;
  FinalLog::init(folder+"/"+log_name);
  if (savePosVel){
    logv.open((folder+"/"+log_name+"_"+m_name+"_v").c_str());
    logd.open((folder+"/"+log_name+"_"+m_name+"_d").c_str());
  }
  finished = finishedBySimTime = false;
  stalls = 0; 
  alreadyStalled = false;
  exitedFromTargetRegion = false;
  alreadyChanged = 1;
  destinationX = waypoints[0][0];
  destinationY = waypoints[0][1];
  m_state = GOING;
  pos->SetColor(GOING_COLOR);
  fx = fy = 0;
  currentWaypoint = 0;
  finalTh = m_th + PI - 2*PI/K_path;
  line1parametersWasNotCalculated          =
  line2parametersWasNotCalculated          =
  toTargetRegionParametersWasNotCalculated =
  circularParametersWasNotCalculated       =
  circularParameters2WasNotCalculated      = true;
  mean_distance = d; var_distance = 0; mean_velocity = 0; var_velocity = 0;
  n_distances = n_velocities = 0;
  init_laser();
}

//Finish robot. Here only display a message for looking with the simulation is still running.
void Robot::finish()
{
  //This message is to see how many robots ended while experimentation scripts are ongoing, then I can see if it is stopped.
  cout << "Robot " << m_id << " finished!" << endl;
  if (savePosVel){
    logv.close();
    logd.close();
  }
}

//Subtract two angle values (in radians). The result value lies between -2 PI and 2 PI. 
double Robot::angDiff(double end, double begin)
{
   double returnMe = end - begin;
  
   if (returnMe > PI)
      returnMe = -(2*PI - returnMe);
   else if (returnMe < -PI)
      returnMe = 2*PI + returnMe;
    
   return returnMe;
}

// Set fx and fy to the attractive force to point (gotoX,gotoY).
// The modulo of (fx,fy) will be Ka.
void Robot::setAttractiveForce(double gotoX, double gotoY)
{
  fx = (gotoX - m_x);
  fy = (gotoY - m_y);
  saturation(fx,fy,Ka);
  #ifdef DEBUG_FORCES
  fv.setAttractiveForces(fx,fy);
  #endif
}

// Add to fx and fy the attractive force to point (gotoX,gotoY).
// The modulo of the force after the addition will be Ka.
void Robot::addAttractiveForce(double gotoX, double gotoY)
{
  double _fx = (gotoX - m_x);
  double _fy = (gotoY - m_y);
  // This constant must be greater than the one used on the circular path to avoid local minima on the line and circle paths' joining.
  saturation(_fx,_fy,1.5*Ka);
  fx += _fx;
  fy += _fy;
  saturation(fx,fy,Ka);
  #ifdef DEBUG_FORCES
  fv.setAttractiveForces(fx,fy);
  #endif
}


//Alter the values of fx and fy, adding repulsion force by target region.
void Robot::targetRegionRepulsionForce()
{
  double fx_ = 0 ,fy_ = 0;
  double influence =  D;
  
  double tzx = waypoints[0][0] - m_x, tzy = waypoints[0][1] - m_y;
  double distance = hypot(tzx,tzy) - D;
  if (distance <= influence)
  {
     fx_ = -Kobs*(1.0/distance - 1.0/(influence*2))*(1.0/pow(distance,2))*(tzx/(distance+D));
     fy_ = -Kobs*(1.0/distance - 1.0/(influence*2))*(1.0/pow(distance,2))*(tzy/(distance+D));
  }
  fx += fx_;
  fy += fy_;
  #ifdef DEBUG_FORCES
  fv.setTargetRepulsiveForces(fx_,fy_);
  #endif
}

//Alter the values of fx and fy, adding repulsion force.
void Robot::obstaclesRepulsionForces()
{  
   double distance;
   double dx = 0;
   double dy = 0;
   double fx_ = 0 ,fy_ = 0;

   const std::vector<meters_t>& scan = laser->GetSensors()[0].ranges;
   uint32_t sample_count = scan.size();
   
   double influence = d;
   double min_scan = influence;
   for(uint i = 0; i < sample_count; i++)
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
     if (savePosVel) logd << min_scan << endl;
   }
   #ifdef DEBUG_FORCES
   fv.setRepulsiveForces(fx_,fy_);
   #endif
}

// Set linear and turning speeds based on the force vector.
// The linear velocity is set to a maximum value.
void Robot::setSpeeds()
{
  if (isHolonomic){
    /* Control for omni robot */
    // linSpeed is repurposed to signify x_speed
    // rotSpeed is repurposed to signify y_speed
    double x_accel = fx;
    double y_accel = fy;
    
    x_accel += -Kdp*linSpeed;
    y_accel += -Kdp*rotSpeed;
    
    linSpeed = linSpeed + x_accel*(TIME_STEP);
    rotSpeed = rotSpeed + y_accel*(TIME_STEP);
    
    // suppress maximum velocity
    double normspeed = hypot(linSpeed,rotSpeed);
    if (normspeed > maxLinSpeed){
      linSpeed = maxLinSpeed*linSpeed/normspeed;
      rotSpeed = maxLinSpeed*rotSpeed/normspeed;
    }
    
    this->pos->SetSpeed(linSpeed, rotSpeed, 0);
    
    if (!finished){
      Velocity veloc = this->pos->GetVelocity();
      double linvec = hypot(veloc.x,veloc.y);
      if (linvec > maxVelocity) 
        maxVelocity = linvec;
      n_velocities++;
      IterativeMeanVariance(linvec, n_velocities, &mean_velocity, &var_velocity);
      if (savePosVel) logv << linvec << endl;
    }
  }
  else{
    double angTarget;
    if (fx == 0 && fy == 0) 
        angTarget = m_th;
    else  
        angTarget = atan2(fy, fx);
    
    double linAccel = Kl*(fx*cos(m_th) + fy*sin(m_th));
    double rotAccel = Kr*(angDiff(angTarget, m_th));
    
    linAccel += -Kdp*linSpeed;
    rotAccel += -Kdp*rotSpeed;
    
    linSpeed = linSpeed + linAccel*(TIME_STEP);
    rotSpeed = rotSpeed + rotAccel*(TIME_STEP);
    
    // suppress maximum velocity
    if (linSpeed > maxLinSpeed) linSpeed = maxLinSpeed;
    
    this->pos->SetXSpeed(linSpeed);
    this->pos->SetTurnSpeed(rotSpeed);
    
    if (!finished){
      Velocity veloc = this->pos->GetVelocity();
      if (veloc.x > maxVelocity) 
        maxVelocity = veloc.x; 
      n_velocities++;
      IterativeMeanVariance(veloc.x, n_velocities, &mean_velocity, &var_velocity);
      if (savePosVel) logv << veloc.x << endl;
    }
  }
}


//Set line 1 parameters just once.
inline void Robot::setLine1PathParameters(double lineangle){
  if (line1parametersWasNotCalculated){
    double ca = cos(lineangle);
    double sa = sin(lineangle);
    double d2sa = (d/2)*sa;
    double d2ca = (d/2)*ca;
    /*
    Corridor variables map:
    
     (X1-d2sa,Y1+d2ca)          (X2-d2sa,Y2+d2ca)
              __________________________
             |                          |
     (X1,Y1) --------------------------- (X2,Y2)
             |__________________________|
     (X1+d2sa,Y1-d2ca)          (X2+d2sa,Y2-d2ca)
    */
    double X1 = waypoints[0][0] + dcurve*ca;
    double Y1 = waypoints[0][1] + dcurve*sa;
    double X2 = waypoints[0][0] + (dcurve + corridorLength)*ca;
    double Y2 = waypoints[0][1] + (dcurve + corridorLength)*sa;
    //setting the parameters used by the vector field generator
    //starting point
    w1x = X2 + d2sa;
    w1y = Y2 - d2ca;
    //ending point
    w2x = X1 + d2sa;  
    w2y = Y1 - d2ca;
    // vector w2 - w1 and its modulus
    w2_w1x = w2x - w1x;
    w2_w1y = w2y - w1y;
    double w2_w1mod = hypot(w2_w1x, w2_w1y);
    w2_w1mod2 = w2_w1mod*w2_w1mod;
    line1parametersWasNotCalculated = false;
  }
}

//Set line 2 parameters just once.
inline void Robot::setLine2PathParameters(double lineangle){
  //Calculate just the exiting waypoint
  double w4x,w4y;
  if (line2parametersWasNotCalculated){
    double ca = cos(lineangle-alpha);
    double sa = sin(lineangle-alpha);
    double d2sa = (d/2)*sa;
    double d2ca = (d/2)*ca;
    /*
    Corridor variables map:
    
     (X1-d2sa,Y1+d2ca)          (X2-d2sa,Y2+d2ca)
              __________________________
             |                          |
     (X1,Y1) --------------------------- (X2,Y2)
             |__________________________|
     (X1+d2sa,Y1-d2ca)          (X2+d2sa,Y2-d2ca)
    */
    double X1 = waypoints[0][0] + dcurve*ca;
    double Y1 = waypoints[0][1] + dcurve*sa;
    double X2 = waypoints[0][0] + (dcurve + corridorLength)*ca;
    double Y2 = waypoints[0][1] + (dcurve + corridorLength)*sa;
    //starting point
    w3x = X1 - d2sa;
    w3y = Y1 + d2ca;
    //ending point
    w4x = X2 - d2sa;
    w4y = Y2 + d2ca;
    // vector w2 - w1 and its modulus
    w4_w3x = w4x - w3x;
    w4_w3y = w4y - w3y;
    double w4_w3mod = hypot(w4_w3x, w4_w3y);
    w4_w3mod2 = w4_w3mod*w4_w3mod;
    line2parametersWasNotCalculated = false;
  }
}

/* 
Line path vector field constructor. Sets fx and fy. Let w1 be the line starting point coordinates and w2 the line ending point coordinates in the arguments explanations below.
Arguments:
  w2_w1x_, w2_w1y_, w2_w1mod2_: x and y coordinates of the vector w2 - w1 and its modulus squared;
  w1x_, w1y_: x and y coordinates of the vector w1.
Return true if the robot arrived next to the ending waypoint w2.
*/
bool Robot::linePathFollowing(double w2_w1x_, double w2_w1y_, double w2_w1mod2_, double w1x_, double w1y_){
  double z_w1x = m_x - w1x_, z_w1y = m_y - w1y_;
  double s = (z_w1x*w2_w1x_ + z_w1y*w2_w1y_)/w2_w1mod2_;
  double Xc;
  if (s >= 1){
    return true;
  }
  else{
    double Xf = atan2(w2_w1y_, w2_w1x_);
    double ep = hypot(z_w1x - s*w2_w1x_, z_w1y - s*w2_w1y_);
    int rho = (w2_w1x_*z_w1y - w2_w1y_*z_w1x >= 0)? 1 : -1 ;
    const double tau = d/5, Xe = PI/2;
    if (ep > tau){
      Xc = Xf - rho*Xe;
    }
    else{
      ep *= rho;
      double power1 = pow(ep/tau,Kexpoent1), power2 = pow(ep,Kexpoent1-1);
      if (isnan(power1)) power1 = 0;
      if (isnan(power2)) power2 = 0;
      double Xd = Xf - Xe*power1;
      Xc = Xd - ((Kexpoent1*Xe*maxLinSpeed)/(Kr*powtauk))*power2*sin(m_th);
    }
  }
  fx = Ka*cos(Xc);
  fy = Ka*sin(Xc);
  #ifdef DEBUG_FORCES
  fv.setAttractiveForces(fx,fy);
  #endif
  return false;
}

/*
Set first circular path parameters just once.
*/
inline void Robot::setCircularPathParameters(){
  if (circularParametersWasNotCalculated){
    cx = waypoints[0][0];
    cy = waypoints[0][1];
    //w1 - c vector
    w1cx = w1x - cx;
    w1cy = w1y - cy;
    r1 = D;
    powrk = pow(r1,Kexpoent2);
    circularParametersWasNotCalculated = false;
  }
}

/*
Set circular parameters for second circular path just once.
*/
inline void Robot::setCircularPathParameters2(int linenumber){
  if (circularParameters2WasNotCalculated){
    double ang = (linenumber - 0.5)*alpha;
    double cb = cos(ang);
    double sb = sin(ang);
    cx = waypoints[0][0] + (waypointDist+r)*cb; 
    cy = waypoints[0][1] + (waypointDist+r)*sb;
    wcx = cx - r*cb;
    wcy = cy - r*sb;
    //w1 - c vector
    w1cx = w3x - cx;
    w1cy = w3y - cy;
    r1 = r;
    powrk = pow(r1,Kexpoent2);
    circularParameters2WasNotCalculated = false;
  }
}

/*
Cicular path vector field constructor. Sets fx and fy. 
Returns true if the robot already reached the final waypoint.
*/
double Robot::circularPathFolowing(){
  double dx = m_x - cx, dy = m_y - cy, dm = hypot(dx, dy);
  double Xc, s = dx*w1cy -  dy*w1cx;
  if (s <= 0 ) return s;
  double gamma = atan2(dx,dy);
  if (dm > 2*r1){
    Xc = gamma - 5*PI/6 + (maxLinSpeed/dm)*sin(m_th - gamma);
  }
  else{
    double power1,power2;
    power1 = pow((dm - r1)/r1,Kexpoent2);
    power2 = pow(dm - r1,Kexpoent2-1);
    if (isnan(power1)) power1 = 0;
    if (isnan(power2)) power2 = 0;
    double Xd = gamma - PI/2. - (PI/3.)*power1;
    Xc = Xd - (maxLinSpeed*sin(m_th - gamma))/(Kr*dm) - (Kexpoent2*maxLinSpeed*PI*power2*cos(m_th-gamma))/(3*powrk*Kr);
  }
  /* Xc is oriented based on the vector (0,1) and clockwise in the original paper. To convert Xc to default orientation (i.e., based on vector (1,0) and counter-clockwise), use PI/2 - Xc.
  */
  fx = Ka*cos(PI/2-Xc);
  fy = Ka*sin(PI/2-Xc);
  #ifdef DEBUG_FORCES
  fv.setAttractiveForces(fx,fy);
  #endif
  return s;
}

/* Follow the line-circle-line vector field.
Reference: Vector field path following for small unmanned air vehicles.
Conference Paper in Proceedings of the American Control Conference · July 2006
DOI: 10.1109/ACC.2006.1657648 */
void Robot::coolPathFollowing(){
  //Calculate the target direction vector angle.
  double ang = atan2(m_y - waypoints[0][1], m_x - waypoints[0][0]);
  if (ang < 0) ang = ang + 2*PI;
  
  //Get the entering lane angle next to ang.
  unsigned int linenumber = (unsigned int) floor(ang/alpha) + 1; 
  double lineangle = linenumber*alpha;


  if (m_state == ENTERING_OUTSIDE){
    setLine1PathParameters(lineangle);
    setCircularPathParameters();
    if(circularPathFolowing() <= 0){
      m_state = ENTERING_LINE1;
      pos->SetColor(ENTERING_LINE1_COLOR);
    }
  }
  else if (m_state == ENTERING_LINE1){
    if (linePathFollowing(w2_w1x, w2_w1y, w2_w1mod2, w1x, w1y)){
      m_state = ENTERING_CURVE;
      pos->SetColor(ENTERING_CURVE_COLOR);
    }
  }
  else if (m_state == ENTERING_CURVE){
    setLine2PathParameters(lineangle);
    setCircularPathParameters2(linenumber);
    circularPathFolowing();
    addAttractiveForce(waypoints[0][0],waypoints[0][1]);
    if ((pho() < waypointDist + epsilon) && !finished)
    {
      finished = true;
      currentWaypoint = 1 + (rand() % NUMBER_OF_WAYPOINTS);
      m_state = LEAVING_CURVE;
      pos->SetColor(LEAVING_CURVE_COLOR);
      numIterationsReachGoal = numIterations;
      numIterations = 0;
      reachingTargetTime = theWorld->SimTimeNow();
    }
  }
  else if (m_state == LEAVING_CURVE){
    double  ended = circularPathFolowing();
    addAttractiveForce(w3x,w3y);
    if (
      ended <= 0 
      /* the condition below is for the case when K is greater than the maximum allowed value for r > 0. In these cases, there are paths to target region, but the curve is reversed, leading to errors. */
      || hypot(m_x - w3x, m_y - w3y) <= epsilonLineFollowing ){
      m_state = LEAVING_LINE2;
      pos->SetColor(LEAVING_LINE2_COLOR);
    }
  }
  else if (m_state == LEAVING_LINE2){
    linePathFollowing(w4_w3x, w4_w3y, w4_w3mod2, w3x, w3y);
  }
}

//Saturate  a vector to a limit modulo, keeping scale.
void Robot::saturation(double &x, double &y, double limit)
{
  double m = hypot(x,y);
  x = limit*x/m;
  y = limit*y/m;
}

//Implements the main loop of robot. 
//Also contain robot controller and 
//probabilistic finite state machine codes
void Robot::mainLoop()
{
  numIterations++;

  //how many times robot stall?
  if (pos->Stalled())
  {
    if (!alreadyStalled)
    {
      stalls++;
      alreadyStalled = true;
    }
  }
  else
  {
    alreadyStalled = false;
  }

  Pose pose = pos->GetPose();
  m_x = pose.x;
  m_y = pose.y;
  m_th = pose.a;
  #ifdef DEBUG_FORCES
    fv.setPosition(m_x,m_y,m_th);
  #endif

  // This conditions serves to give some time before finishing simulation. 
  if (finished && exitedFromTargetRegion)
  {
    //If ITERATION_FOR_CHANGING_COLOR is 1, the robot changes immediately to finished state and saves the log. This is used for generate videos or plotting images, so that the last robot will be black. Use ITERATION_FOR_CHANGING_COLOR = 99 in commonConfig.h when do that. 
    if (alreadyChanged % ITERATION_FOR_CHANGING_COLOR == 0){
      FinalLog::finish();
    }
    alreadyChanged++;
  }

  if (m_state == GOING && pho() <= D){
    m_state = ENTERING_OUTSIDE;
    pos->SetColor(ENTERING_OUTSIDE_COLOR);
  }

  if ((m_state == LEAVING_CURVE || m_state == LEAVING_LINE2 ) && hypot(m_x - waypoints[0][0], m_y - waypoints[0][1]) > D){
    m_state = GOING_OUT;
    pos->SetColor(END_COLOR);
    FinalLog::refresh(numIterationsReachGoal, numIterations, stalls, theWorld->SimTimeNow(), reachingTargetTime, maxVelocity, minDistance, mean_distance, var_distance, n_distances, mean_velocity, var_velocity, n_velocities, theWorld->SimTimeNow() - reachingTargetTime);
    FinalLog::notify_finish();
    exitedFromTargetRegion = true;
    finish();
  }

  if ((testTime - FINISH_TIME < theWorld->SimTimeNow()) && !finishedBySimTime )
  {
    finishedBySimTime = true;
    numIterationsReachGoal = numIterations;
    numIterations = 0;
    FinalLog::refresh_not_at_target(numIterationsReachGoal, numIterations, stalls, maxVelocity, minDistance);
    FinalLog::notify_finish_not_at_target();
    finish();
    FinalLog::finish();
  }

  destinationX = waypoints[currentWaypoint][0];
  destinationY = waypoints[currentWaypoint][1];

  if (m_state == GOING_OUT || m_state == GOING){
    setAttractiveForce(destinationX,destinationY);
  }
  else{
    coolPathFollowing();
  }
  obstaclesRepulsionForces();

  if (m_state == GOING_OUT){
    targetRegionRepulsionForce();
  }


  saturation(fx, fy, maxForce);
  #ifdef DEBUG_FORCES
  fv.setResultantForces(fx,fy);
  #endif
  setSpeeds();
}

void Robot::saveMyLog()
{
  log.open((folder+"/"+m_name).c_str());
  log << m_id << endl
      << numIterationsReachGoal << endl
      << numIterations << endl
      << theWorld->SimTimeNow() << endl
      << setprecision(17)
      << m_y << endl
      << m_x << endl
      << endl;
  log.close();
}

//Pointer to a new robot.
//Every call of this library will create a new robot
//using this pointer.
Robot* robot;


extern "C" int Init(Model *mod, CtrlArgs *args)
{
  vector<string> tokens;
  Tokenize(args->worldfile, tokens); 
  if (tokens.size() < 3){
    cout << endl;
    cout << "Wrong number of arguments." << endl;
    cout << "Usage:" << endl
         << "  coordination.so <config file> <robot id>" << endl;
    exit(1);
  }
  robot = new Robot(tokens[1]); 
  ModelPosition *pmod = (ModelPosition*) mod;
  robot->pos = pmod;
  robot->theWorld = pmod->GetWorld(); //ゴ ゴ ゴ ゴ
  robot->pos->AddCallback(Model::CB_UPDATE, (model_callback_t)PositionUpdate, robot );
  robot->laser = (ModelRanger*)mod->GetChild( "ranger:1" );
  
  robot->laser->Subscribe(); // starts the laser updates
  robot->pos->Subscribe(); // starts the position updates

  robot->init(atoi(tokens[2].c_str()));
  #ifdef DEBUG_FORCES
  robot->pos->AddVisualizer( &robot->fv, true );
  #endif
  return 0;
}

