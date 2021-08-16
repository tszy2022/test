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

  // 向服务器发起连
  if (TcpClient.ConnectToServer("192.168.1.10",4001)==false)
  { printf("TcpClient.ConnectToServer(\"192.168.1.10\",4001) failed,exit...\n"); return -1; }

  unsigned char strbuffer[13];
  //char strbuf1[8] {\000,\000,\000,\000,\000,\000,\000,\b};
  for (int ii=0;ii<50;ii++)
  {

   int i=0;
    memset(strbuffer,0,sizeof(strbuffer));
   while(1)
   {
		i=0;

     TcpClient.Read(strbuffer,sizeof(strbuffer));
	while(i<=12)
		{
		printf("%0x \n",strbuffer[i]);
		i++;
		}
		printf("\n");

   }


    printf("接收：%s\n",strbuffer);
 printf("接收1:");

  printf("/n");
    sleep(1);  // sleep一秒，方便观察程序的运行。
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
