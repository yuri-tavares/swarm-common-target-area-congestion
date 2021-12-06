/*
  This file implement a shared library using functions to draw lines on Stage.
*/

#include "wiseRobot.h"
#include "../common/util.h"

//Define a point
struct _Ponto{
  double px;
  double py;
};

typedef struct _Ponto Ponto;

//class that implement Rects Visualizer
class RectVisualizer: public Visualizer{
  double WPDIST; //radius of the circular target region
  double DangerDist; //distance from the target for EE algorithm.
  double FreeDist; //distance from the target for PCC algorithm.

  //Draw two lines crossing the goal.
  //The angle between these lines is ALFA (specified by file alfa.h)
  void drawIntersection(){
  
    const double alpha = (PI/180.0)*(90 - ALFA/2);
    const double gama =  (PI/180.0)*(90 + ALFA/2);
    
    Ponto reta1[2];
    Ponto reta2[2];
    
    reta1[0].px = -DISTANT_RADIUS; 
    reta1[0].py = 0 + tan(alpha)*(reta1[0].px - 0);
    reta1[1].px = DISTANT_RADIUS;
    reta1[1].py = 0 + tan(alpha)*(reta1[1].px - 0);
    
    reta2[0].px = -DISTANT_RADIUS; 
    reta2[0].py = 0 + tan(gama)*(reta1[0].px - 0);
    reta2[1].px = DISTANT_RADIUS;
    reta2[1].py = 0 + tan(gama)*(reta1[1].px - 0);

    glColor3f(0,0,0);
    
    glBegin( GL_LINES );
      glVertex2f( reta1[0].px + waypoints[0][0], reta1[0].py + waypoints[0][1]);
      glVertex2f( reta1[1].px + waypoints[0][0], reta1[1].py + waypoints[0][1]);
      glVertex2f( reta2[0].px + waypoints[0][0], reta2[0].py + waypoints[0][1]);
      glVertex2f( reta2[1].px + waypoints[0][0], reta2[1].py + waypoints[0][1]);
    glEnd();

    double angle;
    glBegin(GL_LINE_LOOP);
      for (angle=0.0; angle < 2*PI; angle+=10*(PI/180.0))
      {
        glVertex2f(cos(angle) * DangerDist + waypoints[0][0],
                   sin(angle) * DangerDist + waypoints[0][1]);
      }
    glEnd();

    glBegin(GL_LINE_LOOP);
      for (angle=0.0; angle < 2*PI; angle+=10*(PI/180.0))
      {
        glVertex2f(cos(angle) * DISTANT_RADIUS + waypoints[0][0],
                   sin(angle) * DISTANT_RADIUS + waypoints[0][1]);
      }
    glEnd();

    glBegin(GL_LINE_LOOP);
      for (angle=0.0; angle < 2*PI; angle+=10*(PI/180.0))
      {
         glVertex2f(cos(angle)*FreeDist + waypoints[0][0],sin(angle)*FreeDist+ waypoints[0][1]);
      }
    glEnd();

    // circle of target size
    glBegin(GL_LINE_LOOP);
      for (angle=0.0; angle < 2*PI; angle+=10*(PI/180.0))
      {
         glVertex2f(cos(angle)* WPDIST + waypoints[0][0],
                    sin(angle)* WPDIST+ waypoints[0][1]);
      }
    glEnd();
  }
  
  public:
  RectVisualizer(double wpdist):Visualizer("rects","vis_rects"){
    WPDIST = wpdist;
    FreeDist = WPDIST + 0.7;
    DangerDist = WPDIST + 1.5;
  }

  void Visualize( Model* mod, Camera* cam ){
    drawIntersection();
  }

};

RectVisualizer *rect;

extern "C" int Init(Model *mod, CtrlArgs *args){
  vector<string> tokens;
  Tokenize(args->worldfile, tokens); 
  rect = new RectVisualizer(atof(tokens[1].c_str()));
  mod->Subscribe();
  mod->AddVisualizer( rect, true );
  return 0;
}

