/**
  Clients.h possue estruturas de dados, classes e definições para 
  tratar o cliente de envio de mensagens.

  Faz parte do simulador de envio de mensagens entre robôs.

*/

#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <iostream>
#include <sstream>
#include <string>
#include <stdexcept>

#include "connection.h"

using namespace std;

//Definição de cada cliente
typedef struct strClient
{
      int socket;
      bool finished;
} strClient;

//Esta classe possui uma lista de clientes
class ClientList
{
  public:
   // Start up the list, given a socket to listen for incomming connections
   void start(int listener);
   // Start a connection with a client
   void startConnection(int socket);
   // Waits for connections
   void waitConnections();
   // Return a client, given a socket
   strClient returnClient(int socket);
   // Setup the number of robots in the experiment
   void setNumberOfRobots(int number);
   // Remove a client
   void removeClient(strClient client);
   // Qualify a client as a finished one
   void finishClient(strClient client);
   // Quit all clients
   void quit();

   // To simplify the implementation, it is going to be public. But please, do no write on this vector.
   vector<strClient> hearMe;

  private:
   // Send a message to initialize the clients
   void startClients();

   vector<strClient> clients;
   fd_set connections;
   int max;
   int maximumNumberOfRobots;
   int actualNumberOfRobots;

   int listener;
};
