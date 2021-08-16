#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>

// TCP客户端类
class CTcpClient
{
public:
  int m_sockfd;

  CTcpClient();

  // 向服务器发起连接，serverip-服务端ip，port通信端口
  bool ConnectToServer(const char *serverip,const int port);
  // 向对端发送报文
  int  Send(const void *buf,const int buflen);
  // 接收对端的报文
  int  Recv(void *buf,const int buflen);
  //桌面
  int Read(void *buf,const int buflen);


 ~CTcpClient();
};

int main()
{
  CTcpClient TcpClient;

    int a {0};

  // 向服务器发起连
  if (TcpClient.ConnectToServer("192.168.1.10",4001)==false)
  { printf("TcpClient.ConnectToServer(\"192.168.1.10\",4001) failed,exit...\n"); return -1; }
        unsigned char strbuffer[13]={0x08,0x00,0x00,0x04,0x21,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  TcpClient.Send(strbuffer,13);
    while(1)
    {
  //unsigned char strbuffer[13]={0x08,0x00,0x00,0x01,0x11,0x00,0x96,0x00,0x00,0x00,0x00,0x00,0x00};

//unsigned char strbuffer[13]={0x08,0x00,0x00,0x01,0x11,0x00,0x00,0x00,0xc8,0x00,0x00,0x00,0x00};
unsigned char strbuffer[13]={0x08,0x00,0x00,0x01,0x21,0x01,0x01,0x05,0x01,0x05,0x00,0x00,0x00};

unsigned char strbuffer0[13]={0x08,0x00,0x00,0x01,0x21,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00};


//char strbuf1[8] {\000,\000,\000,\000,\000,\000,\000,\b};
   TcpClient.Send(strbuffer,13);
   sleep(1);
     TcpClient.Send(strbuffer0,13);
   sleep(1);
 //    unsigned char strbuffer0[13]={0x08,0x00,0x00,0x01,0x11,0x00,0x00,0x00,0xc8,0x00,0x00,0x00,0x00};
  //TcpClient.Send(strbuffer0,13);
  // sleep(0.5);
    }
}

CTcpClient::CTcpClient()
{
  m_sockfd=0;  // 构造函数初始化m_sockfd
}

CTcpClient::~CTcpClient()
{
  if (m_sockfd!=0) close(m_sockfd);  // 析构函数关闭m_sockfd
}

// 向服务器发起连接，serverip-服务端ip，port通信端口
bool CTcpClient::ConnectToServer(const char *serverip,const int port)
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

int CTcpClient::Send(const void *buf,const int buflen)
{
  return send(m_sockfd,buf,buflen,0);
}

int CTcpClient::Recv(void *buf,const int buflen)
{
  return recv(m_sockfd,buf,buflen,0);
}
int CTcpClient::Read(void *buf,const int buflen)
{
  return read(m_sockfd,buf,buflen);
}


