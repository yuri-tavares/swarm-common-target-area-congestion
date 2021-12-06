/*
  This file implement a shared library using functions to draw lines on Stage.
*/

#include <stage.hh>
#include "commonConfig.h"
#include "ConfigFile.h"
#include "util.h"

using namespace Stg;
using namespace std;

#define PI M_PI

//Define a point
struct _Ponto{
  double px;
  double py;
};

typedef struct _Ponto Ponto;

//class that implement Rects Visualizer
class RegionsVisualizer: public Visualizer{
  private:
    double waypointDist;    // radius of target area.
    double d;   //minimum distance between the robots
    double r;   //r adjusted
    double dcurve;  //distance from the lane to the beginning of the curve.
    double corridorLength; //size of the lanes, before the curved path.
    double D;  //maximum distance from the target to algorithm takes effect.
    double alpha;
    int K;

    //Reads a config file and intializes the private members.
    void readConfigFile(string const& confFileName){
      ConfigFile cf(confFileName);
      try{
        waypointDist = atof(cf.valueOf("s").c_str());
        d = atof(cf.valueOf("d").c_str());
        D = atof(cf.valueOf("D").c_str());
        K = atoi(cf.valueOf("K").c_str());
      }
      catch(string s) {
        cout << "Configuration file is incorret: " << s << endl;
        exit(1);
      }
    }
    
    //Draw target region and corridor
    void drawRegions(){
      double angle;
      int i;
      glColor3f(0,0,0);
      for (i=0, angle=0.0; i < K; i++, angle+=2*PI/K){
        double ca = cos(angle);
        double sa = sin(angle);
        double X1 = waypoints[0][0] + dcurve*ca;
        double Y1 = waypoints[0][1] + dcurve*sa;
        double X2 = waypoints[0][0] + (dcurve + corridorLength)*ca;
        double Y2 = waypoints[0][1] + (dcurve + corridorLength)*sa;
        double d2sa = (d/2)*sa;
        double d2ca = (d/2)*ca;
        /*
        Corridor variables depicting:
        
         (X1-d2sa,Y1+d2ca)          (X2-d2sa,Y2+d2ca)
                  __________________________
                 |                          |
         (X1,Y1) --------------------------- (X2,Y2)
                 |__________________________|
         (X1+d2sa,Y1-d2ca)          (X2+d2sa,Y2-d2ca)
        */
        glBegin(GL_LINES);
          glVertex2f(X1, Y1);
          glVertex2f(X2, Y2);
        glEnd();
        glBegin(GL_LINE_LOOP);
          //d cos(90º+a) = - d sin(a) and d sin(90º+a)  = d cos(a).
          glVertex2f(X1 - d2sa, Y1 + d2ca);
          glVertex2f(X2 - d2sa, Y2 + d2ca);
          //d cos(-90º+a) = d sin(a) and d sin(-90º+a)  = - d cos(a).
          glVertex2f(X2 + d2sa, Y2 - d2ca);
          glVertex2f(X1 + d2sa, Y1 - d2ca);
        glEnd();
              
        double cb = cos(angle - (2*PI/K));
        double sb = sin(angle - (2*PI/K));
        double d2sb = (d/2)*sb;
        double d2cb = (d/2)*cb;
        double X4 = waypoints[0][0] + dcurve*cb - d2sb, 
               Y4 = waypoints[0][1] + dcurve*sb + d2cb; //previous angle, pointing up
        double cx = X4 - r*sb, cy = Y4 + r*cb;
        //semicircle for turning
        glBegin(GL_LINE_STRIP);
          double ang, angB =  PI/2 + angle, angE = 3*PI/2 + angle - 2*PI/K;
          for (ang=angB; ang <= angE; ang+=(angE - angB)/53)
          {
             glVertex2f(cos(ang)*r + cx, sin(ang)*r + cy);
          }
        glEnd();
      } 
      
      // circle of target size
      glBegin(GL_LINE_LOOP);
        for (angle=0.0; angle < 2*PI; angle+=10*(PI/180.0))
        {
           glVertex2f(cos(angle)*waypointDist + waypoints[0][0],
                      sin(angle)*waypointDist + waypoints[0][1]);
        }
      glEnd();
      
      // circle of the algorithm effect.
      glBegin(GL_LINE_LOOP);
        for (angle=0.0; angle < 2*PI; angle+=10*(PI/180.0))
        {
           glVertex2f(cos(angle)*D + waypoints[0][0],
                      sin(angle)*D + waypoints[0][1]);
        }
      glEnd();
    }
  
  public:
    RegionsVisualizer(string const& cfgfilename):Visualizer("reg","vis_reg"){
      readConfigFile(cfgfilename);
      alpha = 2*PI/K;
      r = (waypointDist * sin(alpha / 2.) - d/2.) / (1 - sin(alpha / 2.));
      dcurve = sqrt(pow(waypointDist + r,2) - pow(d/2. + r,2));
      corridorLength = D - dcurve;
    }
  
    void Visualize( Model* mod, Camera* cam ){
      drawRegions();
    }
};

RegionsVisualizer* rect;

extern "C" int Init(Model *mod, CtrlArgs *args){
  vector<string> tokens;
  Tokenize(args->worldfile, tokens); 
  rect = new RegionsVisualizer(tokens[1]);
  mod->Subscribe();
  mod->AddVisualizer( rect, true );
  return 0;
}
