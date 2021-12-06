/*
Implementation of commom members to the class WiseRobots that uses limitation in region
by two lines.
Every test with region delimited by two lines uses these functions.
If happens that some of the methods in this file is changed to
a specific test, it must to be eliminated from here and put on
specific test.

This file matchs with regionMethods.cpp. But only implementation of common
methods are here.

Note: I think that using some type of override, or abstract class can 
solve this. But I don't think that this project will grow so much.
*/

//Calculate the angle between the vector from this 
//robot to goal and the vector from this robot and another robot
double WiseRobot::calcAngTarget(double m_x, double m_y, double tx, double ty)
{
   double th;
   double pho = atan2(m_y - destineY, m_x - destineX);
   double x = m_x;
   double y = m_y;
   double xn, yn;
   double ang;
   
   //Comentário em relação ao código original de Soriano:
   //Esta parte foi modificada (reduzida). Retirei algumas condições desnecessárias.
   //Como eu vi que todos os cálculos apenas incrementam PI/2 a pho e
   //notei que atan2 retorna valores entre -PI e PI, removendo as condições fica
   //a seguinte linha
   th = PI/2 + pho;
   

   // The inverse of the matrix [cos(th) -sin(th) x; sin(th) cos(th) y; 0 0 1]
   xn = cos(th)*tx + sin(th)*ty - sin(th)*y-x*cos(th);
   yn = -sin(th)*tx + cos(th)*ty - cos(th)*y+x*sin(th);

   ang = atan2(yn, xn);

   return ang;
}

//calculate the distance between goal and the goal on the received message
double WiseRobot::distWaypoint(double *msg)
{
   return hypot(destineX - msg[MSG_POS_WAYPOINT_X],destineY - msg[MSG_POS_WAYPOINT_Y]);
}

