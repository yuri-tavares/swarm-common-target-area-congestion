#include "connection.h"

//Given a message, compute the distance between the sender and the receiver
double Connection::dist(double m_x, double m_y, double *msg)
{
   return sqrt(pow(m_x - msg[MSG_POS_MY_X],2) + pow(m_y - msg[MSG_POS_MY_Y],2));
}

//Receive a msg. With received correctly, returns true
bool Connection::receiveMsg(double m_x, double m_y, double m_id, double *msg, int size_msg){
  fd_set readfds;
  struct timeval pooling;
  
  struct sockaddr sender;
  socklen_t sizeSender = sizeof(struct sockaddr);
  
  FD_ZERO(&readfds);
  readfds = sockets;
  pooling.tv_sec = 0;
  pooling.tv_usec = 0;

  select(sock + 1, &readfds, NULL, NULL, &pooling);

  if (FD_ISSET(sock, &readfds)){
     if(recvfrom(sock, msg, sizeof(double)*(size_msg), 0, &sender, &sizeSender) <= 0)
     {
         perror("recvfrom:");
         return false;
     }
     else return (dist(m_x,m_y,msg) < commArea && m_id != (int) msg[MSG_POS_MY_ID]);
  }
 else return false;
}

//send a msg
void Connection::sendMsg(double *msg, int size_msg){
  if (sendto(sock, msg, sizeof(double)*size_msg, 0, (struct sockaddr *)&sockAddr, sizeof(struct sockaddr)) <= 0)
  {
    perror("sendto: ");
  }
  else
  {
    numMessages++;
  } 
}

//initialize the informations for connection
void Connection::init_connection(){
   numMessages = 0;
   int yes = 1;
   // Let's start with the communication mechanism
   if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) == -1) 
   {
      perror("socket");
      exit(1);
   }
   sockAddr.sin_family = AF_INET;     // host byte order
   sockAddr.sin_port = htons(PORT); // short, network byte order
   //pathAddr.sin_addr = *((struct in_addr *)he->h_addr);
   sockAddr.sin_addr.s_addr = inet_addr(BROADCAST_ADDRESS);
   memset(sockAddr.sin_zero, '\0', sizeof sockAddr.sin_zero);
   // To deal with the error "Address already in use"
   if (setsockopt(sock,SOL_SOCKET,SO_REUSEADDR,&yes,sizeof(int)) == -1)
   {
      perror("setsockopt");
      exit(1);
   }
   // this call allows broadcasting
   if (setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &yes,
                  sizeof(int)) == -1)
   {
      perror("setsockopt (SO_BROADCAST)");
      exit(1);
   }
   if (bind(sock, (sockaddr *)&sockAddr, sizeof(sockaddr)) < 0)
   {
      perror("bind: ");
      exit(1);
   }
   FD_ZERO(&sockets);
   FD_SET(sock, &sockets);
   int tmp;
   receiveInt(sockServer, &tmp);
   connectClient(SERVER_ADDRESS, SERVERPORT, sockServer);
}

//informs the server that the robot using this class end your task 
void Connection::finish(int m_id, int numIterationsEnter, int numIterationsExit, int stalls){
  sendSixInts(sockServer, FINISHED, m_id, numMessages, numIterationsEnter, numIterationsExit, stalls);
}

// Send an integer
void sendInt(int connection, int number)
{
   number = htonl(number);
   send(connection, &number, sizeof(number), 0);
}

// Receive an integer
void receiveInt(int connection, int *number)
{
   recv(connection, number, sizeof(number), 0);
   *number = ntohl(*number);
}

// Send four integers
void sendFourInts(int connection, int n1, int n2, int n3, int n4)
{
   int msg[4];
   msg[0] = htonl(n1);
   msg[1] = htonl(n2);
   msg[2] = htonl(n3);
   msg[3] = htonl(n4);
   send(connection, msg, sizeof(int)*4, 0);
}

// Receive four integers
void receiveFourInts(int connection, int &n1, int &n2, int &n3, int &n4)
{
   int msg[4];

   recv(connection, msg, sizeof(int)*4, 0);
   n1 = ntohl(msg[0]);
   n2 = ntohl(msg[1]);
   n3 = ntohl(msg[2]);
   n4 = ntohl(msg[3]);
}

// Send five integers
void sendFiveInts(int connection, int n1, int n2, int n3, int n4, int n5)
{
   int msg[5];
   msg[0] = htonl(n1);
   msg[1] = htonl(n2);
   msg[2] = htonl(n3);
   msg[3] = htonl(n4);
   msg[4] = htonl(n5);
   send(connection, msg, sizeof(int)*5, 0);
}

// Send five integers
void sendSixInts(int connection, int n1, int n2, int n3, int n4, int n5, int n6)
{
   int msg[6];
   msg[0] = htonl(n1);
   msg[1] = htonl(n2);
   msg[2] = htonl(n3);
   msg[3] = htonl(n4);
   msg[4] = htonl(n5);
   msg[5] = htonl(n6);
   send(connection, msg, sizeof(int)*6, 0);
}

// Receive five integers
void receiveSixInts(int connection, int &n1, int &n2, int &n3, int &n4, int &n5, int &n6)
{
   int msg[6];

   recv(connection, msg, sizeof(int)*6, 0);
   n1 = ntohl(msg[0]);
   n2 = ntohl(msg[1]);
   n3 = ntohl(msg[2]);
   n4 = ntohl(msg[3]);
   n5 = ntohl(msg[4]);
   n6 = ntohl(msg[5]);
}

// Receive five integers
void receiveFiveInts(int connection, int &n1, int &n2, int &n3, int &n4, int &n5)
{
   int msg[5];

   recv(connection, msg, sizeof(int)*5, 0);
   n1 = ntohl(msg[0]);
   n2 = ntohl(msg[1]);
   n3 = ntohl(msg[2]);
   n4 = ntohl(msg[3]);
   n5 = ntohl(msg[4]);
}

// Create a client
void connectClient(const char *host, short port, int &sockfd)
{
   sockaddr_in serverAddr;
   hostent *hp;

   // Traduz nome do host para endereço IP
   hp = gethostbyname(host);
   if (!hp)
   {
      cerr << "client: unknown host\n";
      exit(1);
   }

   // Vamos usar TCP/IP
   serverAddr.sin_family = AF_INET;
   // Configurando porta do servidor
   serverAddr.sin_port = htons(port);
   // Configurando endereço do servidor
   bcopy(hp->h_addr, (char *)&serverAddr.sin_addr, hp->h_length);
   // Zera o resto da estrutura
   memset(&(serverAddr.sin_zero), '\0', 8);
   
   // Abertura ativa
   if ((sockfd = socket(PF_INET, SOCK_STREAM, 0)) < 0)
   {
      perror("client: sockfd");
      exit(1);
   }
   if (connect(sockfd, (sockaddr *)&serverAddr, sizeof(sockaddr)) < 0)
   {
      perror("client: connect");
      close(sockfd);
      exit(1);
   }
}
