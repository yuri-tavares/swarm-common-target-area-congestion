#include "forcevisualizer.h"

//this function must to be used every time the robot changes your position 
void ForceVisualizer::setPosition(double m_x, double m_y, double m_th){
  this->m_x = m_x;
  this->m_y = m_y; 
  this->m_th = m_th; 
}

//sets repulsive force value
void ForceVisualizer::setRepulsiveForces(double fxrep, double fyrep){
  this->fxrep = fxrep; 
  this->fyrep = fyrep;  
}

//sets attractive force value
void ForceVisualizer::setAttractiveForces(double fxatt, double fyatt){
  this->fxatt = fxatt;
  this->fyatt = fyatt;
}

//sets boundary line repulsive force. This force concerns a artificial force implied by the boundary line
void ForceVisualizer::setLineRepulsiveForces(double fxlin, double fylin)
{
  this->fxlin = fxlin;
  this->fylin = fylin;
}

//resultant force values
void ForceVisualizer::setResultantForces(double fxR, double fyR)
{
  this->fxR = fxR;
  this->fyR = fyR;
}

//sets repulsive force value
void ForceVisualizer::setSpeed(double vx){
  this->vx = vx; 
}

//This constructor. Initializes forces with zeros.
ForceVisualizer::ForceVisualizer():Visualizer("rects","vis_rects"){
  fxrep = fyrep =
  fxatt = fyatt =
  fxlin = fylin = 0;
}

//Transformation used to plot forces over robot view point
Point ForceVisualizer::T(double x, double y){
  Point p;
  p.x = x*cos(-m_th) - y*sin(-m_th);
  p.y = x*sin(-m_th) + y*cos(-m_th);
  return p;
}

//Draw a vertex relative to robot's position
inline void ForceVisualizer::_glVertex2f(double x, double y){
  Point p = T(x,y);
  glVertex2f(p.x,p.y);
}

//Method to Visualize forces. Inherited from Stage API.
void ForceVisualizer::Visualize( Model* mod, Camera* cam ){
  glBegin( GL_LINES );
    #ifdef SHOW_REPULSIVE
    glColor3f(0,0,1); //Azul
    _glVertex2f(0,0);
    _glVertex2f(fxrep,fyrep);
    #endif
    #ifdef SHOW_ATTRACTIVE
    glColor3f(1,0,0); //Vermelho
    _glVertex2f(0,0);
    _glVertex2f(fxatt,fyatt);
    #endif
    #ifdef SHOW_LINE_REPULSIVE
    glColor3f(0,1,0); //Verde
    _glVertex2f(0,0);
    _glVertex2f(fxlin,fylin);
    #endif
    #ifdef SHOW_RESULTANT
    glColor3f(0,0,0); //Preto
    _glVertex2f(0,0);
    _glVertex2f(fxR,fyR);
    #endif
    #ifdef SHOW_SPEED
    glColor3f(0.5,0.5,0.5); //
    _glVertex2f(0,0);
    _glVertex2f(vx * cos(m_th), vx * sin(m_th));
    #endif
  glEnd();  
}
