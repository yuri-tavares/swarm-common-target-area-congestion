/*
  Implements a server to send and rceive messages between robots.
  Also terminate execution of experiments when every one finish their tasks.
*/

#include <iostream>
#include <fstream>
#include <cmath>
#include <cstdlib>
#include <signal.h>
#include "connection.h"
#include "clients.h"

using namespace std;

int listener; // Socket that waits for incoming connections
int quitListener;
vector<int> quitSockets;
ClientList clients;
ofstream logFile;
int nFinished = 0;
int maximumNumberOfRobots;

void quitServer(int signal)
{
   clients.quit();
   cout << "Interrupted... Good bye!" << endl;
   exit(0);
}

//Control the end of experimentation, create final log (log containing information 
//about experiment) and 
void dataClient(strClient client)
{
   int header;
   int robot;
   static int numMessages = 0; // Total of sent messages
   static int numTotalIterationsReachGoal = 0; // Total number of iterations
   static int numTotalIterationsExitGoal = 0; // Total number of iterations
   static int numMaxIterationsReachGoal = 0; //Maximum number of iterations to reach the goal
   static int numMaxIterationsExitGoal = 0; //Maximum number of iterations to exit from goal
   static int numTotalStalls = 0; //number of times that the robots stalled
   time_t timeStamp;
   int numMessagesRobot = 0;
   int numIterationsRobotReach = 0;
   int numIterationsRobotExit = 0;
   int numStalls = 0;

   receiveSixInts(client.socket, header, robot, numMessagesRobot, numIterationsRobotReach, numIterationsRobotExit, numStalls);

   time(&timeStamp);

   if (header == FINISHED)
   {   
      #ifdef SERVER_FINISH_OUTPUT
      cout << timeStamp << "- " << "Robot " << robot << " finished" << endl;
      #endif
      numTotalStalls += numStalls;
      numMessages += numMessagesRobot;
      numTotalIterationsReachGoal += numIterationsRobotReach;
      numTotalIterationsExitGoal += numIterationsRobotExit;
      if (numIterationsRobotReach > numMaxIterationsReachGoal)
	 numMaxIterationsReachGoal = numIterationsRobotReach;
      if (numIterationsRobotExit > numMaxIterationsExitGoal)
	 numMaxIterationsExitGoal = numIterationsRobotExit;

      clients.finishClient(client);
      nFinished++;
   }
   else
   {
      if (!client.finished)
      {
         #ifdef SERVER_FINISH_OUTPUT
	 cout << timeStamp << "- " << "Connection lost with one robot" << endl;
         #endif
	 clients.removeClient(client);
	 nFinished++; // The other robots might finish the simulation
      }
      else
      {
	 clients.removeClient(client);
      }
   }

   if (nFinished == maximumNumberOfRobots)
   {

      clients.quit();
      #ifdef SERVER_FINISH_OUTPUT
      cout << "All done... Good bye!" << endl;
      #endif
      logFile << numTotalIterationsReachGoal + numTotalIterationsExitGoal << endl;
      logFile << numMaxIterationsReachGoal + numMaxIterationsExitGoal<< endl;
      logFile << numMessages << endl;
      logFile << numTotalIterationsReachGoal << endl
              << numTotalIterationsExitGoal << endl;
      logFile << numMaxIterationsReachGoal << endl
              << numMaxIterationsExitGoal<< endl;
      logFile << numTotalStalls << endl;

      logFile.close();

      system("killall stage server");
      exit(0);
   }
}

//creates a new client
void newClient()
{
   socklen_t addrSize;
   struct sockaddr_in clientAddr;
   int newClient;

   addrSize = sizeof(clientAddr);

   if ((newClient = accept(listener, (struct sockaddr *)&clientAddr,
			   &addrSize)) == -1) 
      perror("accept");
   else
   {
      clients.startConnection(newClient);
   }
  
}

// Create a server
void connectServer(short port, int &sock)
{
   sockaddr_in myAddr;
   int yes = 1;

   myAddr.sin_family = AF_INET;
   myAddr.sin_port = htons(port);
   myAddr.sin_addr.s_addr = INADDR_ANY;
   memset(&(myAddr.sin_zero), '\0', 8);

   // Creating socket
   if ((sock = socket(PF_INET, SOCK_STREAM, 0)) < 0)
   {
      perror("servidor: socket");
      exit(1);
   }

   // To deal with the error "Address already in use"
   if (setsockopt(sock,SOL_SOCKET,SO_REUSEADDR,&yes,sizeof(int)) == -1)
   {
      perror("setsockopt");
      exit(1);
   }

   if (bind(sock, (sockaddr *)&myAddr, sizeof(sockaddr)) < 0)
   {
      perror("servidor: bind");
      exit(1);
   }

   listen(sock, MAX_PENDING);
}

int main(int argc, char **argv)
{
   strClient client;
   struct tm *timeinfo;
   time_t rawtime;
   ostringstream date;

   (void) signal(SIGINT, quitServer);
   (void) signal(SIGTERM, quitServer);

   if (argc != 2)
   {
      cerr << "Invalid parameters" << endl;
      cerr << "Use: " << endl;
      cerr << "server [number of robots]" << endl;
      exit(1);
   }
   else
   {
      maximumNumberOfRobots = atoi(argv[1]);
      clients.setNumberOfRobots(maximumNumberOfRobots);
   }

   time(&rawtime);
   timeinfo = localtime(&rawtime);
   
   date << timeinfo->tm_year + 1900 << "-" << timeinfo->tm_mon + 1 << "-" << timeinfo->tm_mday << "--" << timeinfo->tm_hour << "-" << timeinfo->tm_min << "--" << "token";

   logFile.open("log");

   connectServer(SERVERPORT, listener);
   clients.start(listener);

   while(1)
   {
      clients.waitConnections();

      //search for data in the existings connections
      for(unsigned int i = 0; i < clients.hearMe.size(); i++)
      {
	 client = clients.hearMe.at(i);

	 if (client.socket == listener) // deals with new connections
	 {
	    newClient();
	 }
	 else
	 {
	    dataClient(client);
	 }
      }
   }
   
   close(listener);
}
