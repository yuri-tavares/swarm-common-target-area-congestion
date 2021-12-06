/*
  This class encapsulates information about message sending, 
  server/client function to enable communication among robots.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <getopt.h>
#include <math.h>
#include <netdb.h>
#include <fstream>
#include <iostream>
using namespace std;

#ifndef _CONNECTION_H_
#define _CONNECTION_H_

#define MAX_PENDING 20 // Max number of allowed connections, for each port

#define FINISHED 1

#define SERVERPORT 5001 // Port of the token server, used to send token information
#define QUITPORT 5004 // Port of the token server, used to stop all robots in the end of the simulation

#define SERVER_ADDRESS "localhost" // Address of the token server

// Send an integer
void sendInt(int connection, int number);
// Receive an integer
void receiveInt(int connection, int *number);
// Send four integers
void sendFourInts(int connection, int n1, int n2, int n3, int n4);
// Receive four integers
void receiveFourInts(int connection, int &n1, int &n2, int &n3, int &n4);
// Send five integers
void sendFiveInts(int connection, int n1, int n2, int n3, int n4, int n5);
// Receive five integers
void receiveFiveInts(int connection, int &n1, int &n2, int &n3, int &n4, int &n5);
// Send six integers
void sendSixInts(int connection, int n1, int n2, int n3, int n4, int n5, int n6);
// Receive six integers
void receiveSixInts(int connection, int &n1, int &n2, int &n3, int &n4, int &n5, int &n6);
// Create a server
int connectServer(int &sockfd, short port);
// Create a client
void connectClient(const char *host, short port, int &sockfd);


#define PORT 5002
#define BROADCAST_ADDRESS "127.255.255.255"

//Area (in meters) of robot's communication radius
#define commArea 3

//Index for each field of message vector
#define MSG_POS_TYPE       0 //Type of message
#define MSG_POS_MY_X       1 //X coordinate of robot position
#define MSG_POS_MY_Y       2 //Y coordinate of robot position
#define MSG_POS_WAYPOINT_X 3 //X coordinate of robot's waypoint 
#define MSG_POS_WAYPOINT_Y 4 //Y coordinate of robot's waypoint 
#define MSG_POS_MY_ID      5 //Robot's id


//This class encapsulates information concerning message sending handling.
//Every robot have this class
class Connection{
  private:
    //number of messages sent
    int numMessages;
    
    //this socket number
    int sock;
    
    //number of the server socket
    int sockServer;

    //set of file descriptors. This set is used to see if the socket associated to 
    //this connection was refreshed
    fd_set sockets;

    //Struct to make connection
    struct sockaddr_in sockAddr;

    //Given a message, compute the distance between the sender and the receiver
    double dist(double m_x, double m_y, double *msg);

    #ifdef MESSAGES_LOG
    //Log of messages sent
    ofstream messagesLog;
    #endif
  public:

    //Receive a msg. With received correctly, returns true
    bool receiveMsg(double m_x, double m_y, double m_id, double *msg, int size_msg);

    //send a msg
    void sendMsg(double *msg, int size_msg);

    //initialize the informations for connection
    void init_connection();

    //informs the server that the robot using this class end your task
    void finish(int m_id, int numIterationsEnter, int numIterationsExit, int numStalls);
};

#endif
