#include "clients.h"

// Qualify a client as a finished one
void ClientList::finishClient(strClient client)
{
   // There must have a faster way to do this, but I am a bit afraid to play with pointers

   vector<strClient>::iterator iterator;

   iterator = clients.begin();

   while(iterator != clients.end())
   {
      if (iterator->socket == client.socket)
      {
	 iterator->finished = true;

         return;
      }

      iterator++;
   }
}

// Remove a client
void ClientList::removeClient(strClient client)
{
   vector<strClient>::iterator iterator;

   iterator = clients.begin();

   while(iterator != clients.end())
   {
      if (iterator->socket == client.socket)
      {
         FD_CLR(iterator->socket, &connections);
         clients.erase(iterator);
         close(client.socket);

         return;
      }

      iterator++;
   }
}

// Start up the list, given a socket to listen for incomming connections
void ClientList::start(int listener)
{
   strClient listenerClient;

   listenerClient.socket = listener;

   max = listener;

   FD_ZERO(&connections);
   FD_SET(listener, &connections);

   clients.push_back(listenerClient);

   actualNumberOfRobots = 0;

   this->listener = listener;
}

// Setup the number of robots in the experiment
void ClientList::setNumberOfRobots(int number)
{
   maximumNumberOfRobots = number;
}

// Start a connection with a client
void ClientList::startConnection(int socket)
{
   strClient newClient;

   newClient.socket = socket;
   newClient.finished = false;

   clients.push_back(newClient);

   FD_SET(socket, &connections);

   if (newClient.socket > max)
      max = newClient.socket;

   actualNumberOfRobots++;

   if (actualNumberOfRobots == maximumNumberOfRobots)
      startClients();
}

// Send a message to initialize the clients
void ClientList::startClients()
{
   for(unsigned int i = 0; i < clients.size(); i++)
   {
      if (clients.at(i).socket != listener)
	 sendInt(clients.at(i).socket, 0);
   }
}
   
// Quit all clients
void ClientList::quit()
{
   for(unsigned int i = 0; i < clients.size(); i++)
   {
      close(clients.at(i).socket);
   }
}

// Waits for connections
void ClientList::waitConnections()
{
   fd_set read;

   FD_ZERO(&read); //is it necessary?

   read = connections;

   if (select(max+1, &read, NULL, NULL, NULL) == - 1)
   {
      perror("select");
      exit(1);
   }

   hearMe.clear();

   for(int i = 0; i <= max; i++)
   {
      if (FD_ISSET(i, &read))
      {
	 hearMe.push_back(returnClient(i));
      }
   }
}

// Return a client, given a socket
strClient ClientList::returnClient(int socket)
{
   vector<strClient>::iterator iterator;

   iterator = clients.begin();

   while(iterator != clients.end())
   {
      if (iterator->socket == socket)
	 return *iterator;

      iterator++;
   }

   // ERROR!
   //return -1;
   return *(clients.begin());
}
