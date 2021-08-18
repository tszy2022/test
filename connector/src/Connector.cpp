#include "Connector.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>


Connector::Connector()
{
     m_sockfd=0;
}

Connector::~Connector()
{
    if (m_sockfd!=0) close(m_sockfd);
}
void Connector::print()
{
    printf("hello,world");
}
bool Connector::ConnectToServer(const char *serverip,const int port)
{
  m_sockfd = socket(AF_INET,SOCK_STREAM,0); // 创建客户端的socket

  struct hostent* h; // ip地址信息的数据结构
  if ( (h=gethostbyname(serverip))==0 )
  { close(m_sockfd); m_sockfd=0; return false; }

  // 把服务器的地址和端口转换为数据结构
  struct sockaddr_in servaddr;
  memset(&servaddr,0,sizeof(servaddr));
  servaddr.sin_family = AF_INET;
  servaddr.sin_port = htons(port);
  memcpy(&servaddr.sin_addr,h->h_addr,h->h_length);

  // 向服务器发起连接请求
  if (connect(m_sockfd,(struct sockaddr *)&servaddr,sizeof(servaddr))!=0)
  { close(m_sockfd); m_sockfd=0; return false; }

  return true;
}

int Connector::Send(const void *buf,const int buflen)
{
  return send(m_sockfd,buf,buflen,0);
}

int Connector::Recv(void *buf,const int buflen)
{
  return recv(m_sockfd,buf,buflen,0);
}
int Connector::Read(void *buf,const int buflen)
{
  return read(m_sockfd,buf,buflen);
}
int Connector::init()
{
   if (ConnectToServer("192.168.1.10",4001)==false)
  { printf("TcpClient.ConnectToServer(\"192.168.1.10\",4001) failed,exit...\n"); return -1; }
    unsigned char strbuffer[13]={0x08,0x00,0x00,0x04,0x21,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    Send(strbuffer,13);
unsigned char strbuffer0[13]={0x08,0x00,0x00,0x01,0x21,0x01,0x01,0x05,0x01,0x05,0x00,0x00,0x00};

unsigned char strbuffer1[13]={0x08,0x00,0x00,0x01,0x21,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

unsigned char strbuffer2[13]={0x08,0x00,0x00,0x01,0x21,0x01,0x01,0x05,0x01,0x05,0x00,0x00,0x00};

//char strbuf1[8] {\000,\000,\000,\000,\000,\000,\000,\b};
   Send(strbuffer0,13);
   sleep(1);
    Send(strbuffer1,13);
   sleep(1);
    Send(strbuffer2,13);
   printf("initialization completed.");
}

void Connector::printall()//打印一次所有信息，调试专用函数
{
}
void Connector::unpack_all() //打印一次所有信息，调试专用函数
{
     for (int count{ 0 }; count <= 10; ++count)
        {
            printf("%d",count);
        }

}

