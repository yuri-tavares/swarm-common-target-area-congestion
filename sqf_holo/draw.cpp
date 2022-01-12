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
  double attribute;  //radius of the circular target region

  void drawIntersection(){
    glColor3f(0,0,0);
    double angle;
    glBegin(GL_LINE_LOOP);
      for (angle=0.0; angle < 2*PI; angle+=10*(PI/180.0))
      {
         glVertex2f(cos(angle)*DISTANT_RADIUS + waypoints[0][0],
                    sin(angle)*DISTANT_RADIUS+ waypoints[0][1]);
      }
    glEnd();
    
    // circle of target size
    glBegin(GL_LINE_LOOP);
      for (angle=0.0; angle < 2*PI; angle+=10*(PI/180.0))
      {
         glVertex2f(cos(angle)*attribute + waypoints[0][0],
                    sin(angle)*attribute + waypoints[0][1]);
      }
    glEnd();
  }
  
  public:
  RectVisualizer(double wpdist):Visualizer("rects","vis_rects"){
    attribute = wpdist;
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

