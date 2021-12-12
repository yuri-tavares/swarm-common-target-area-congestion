/*
  Creates automatically scenarios for stage.
  This program generates random positions near the goal (X=0,Y=0)
  for a specified number of robots.
  
  Also it allows specifies the probability to a robot change from WAIT state to IMPATIENT state.
*/

#include <iostream>
#include <fstream>
#include <vector>
#include <cstdlib>
#include <math.h>
#include <string.h>
#include "../common/commonConfig.h"

using namespace std;

#define PI M_PI

typedef struct coord
{
      double x;
      double y;
      double theta;
} coord;

bool intersect(coord newCoord, vector<coord> &history, double mindist)
{
   for(unsigned int i = 0; i < history.size(); i++)
   {
      if (sqrt(pow(newCoord.x - history.at(i).x,2) + pow(newCoord.y - history.at(i).y,2)) < mindist)
        return true;
   }
   return false;
}

void rand(coord &newCoord, vector<coord> &history, double minRadius, double maxRadius, double mindist)
{
   double r;
   double pho;
   double theta;

   while(true)
   {
      r = (   (double)rand() / ((double)(RAND_MAX)+(double)(1)) );
      pho = minRadius + r*(maxRadius - minRadius); 
      r = (   (double)rand() / ((double)(RAND_MAX)+(double)(1)) );
      theta = 0 + r*2*PI;

      newCoord.x = pho*cos(theta);
      newCoord.y = pho*sin(theta);
      newCoord.theta = atan2(0 - newCoord.y, 0 - newCoord.x); 

      newCoord.x = waypoints[0][0] + newCoord.x;
      newCoord.y = waypoints[0][1] + newCoord.y;

      if (!intersect(newCoord, history, mindist))
        break;
   }
}

int main(int argc, char **argv)
{
   if (argc < 5)
   {
      cerr << "Invalid parameters" << endl;
      cerr << "Use: " << endl;
      cerr << "createScenario <file> <numRobots> <log> <target_size> [video]" << endl;
      cerr << "command issued: '";
      for(int j = 0; j < argc; j++){
        cerr << argv[j] << " ";
      }
      cerr << "'" << endl;
      exit(1);
   }

   ofstream output((string(argv[1])+".world").c_str());
   int numRobots = atoi(argv[2]);
   double TARGET_SIZE = atof(argv[4]);
   vector<coord> history;
   coord tmp;
   int numRobot = 0;

   srand(time(NULL));

   output << "# defines 'map' object used for floorplans" << endl
          << "include \"map.inc\"" << endl << endl


          << "# defines sick laser" << endl
          << "include \"sick.inc\"" << endl << endl
          << "# defines Pioneer-like robots" << endl
          << "include \"pioneer.inc\"" << endl << endl
          << "# set the resolution of the underlying raytrace model in meters" << endl
          << "resolution 0.1" << endl
          << "" << endl
          << "speedup -1" << endl
          << "" << endl
          << "# configure the GUI window" << endl
          << "window" << endl
          << "( " << endl
          << "  size [ 591.000 638.000 ] " << endl
          << "  center [ " << waypoints[0][0] <<  " " << waypoints[0][1] << " ] " << endl
          << "  show_data 1" << endl;

          if (argc >= 6){
            if (strcmp(argv[5],"video") == 0){
              output << "  screenshots 1" << endl;
            }
          }

   output << ")" << endl
          << "define robot pioneer2dx" << endl
          << "(" << endl
          << "  sicklaser ()" << endl
          << "  size [0.44 0.44 0.44]" << endl
          << "  localization \"gps\"" << endl
          << "  localization_origin [ 0 0 0 0 ]" << endl
          << "  drive \"omni\"" << endl
          << ")" << endl << endl
          << "model" << endl
          << "(" << endl
          << "  name \"toDraw\""<< endl
          << "  size [0.001 0.001 0.001]"<< endl
          << "  pose [0 0 0 0]"<< endl
          << "  ctrl \"draw.so " << TARGET_SIZE << "\""<< endl
          << "  obstacle_return 0" << endl
          << ")"<< endl << endl;
   
   for(int i = 0; i < numRobots; i++)
   {
      rand(tmp, history, DISTANT_RADIUS, DISTANT_RADIUS + 8., 1.0); 
      history.push_back(tmp);
      output << "robot" << endl
             << "(" << endl
             << "  name \"robot" << numRobot << "\"" << endl
             << "  color \"red\"" << endl
             << "  pose [" << tmp.x << " " << tmp.y << " 0 " << "0 " << "]" << endl
             << "  ctrl \"coordination.so " << i << " " << numRobots << " " << TARGET_SIZE << " " << argv[1] << " " << argv[3] << "\"" << endl
             << ")" << endl << endl;

      numRobot++;

   }

   output.close();

}
