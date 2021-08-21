#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
 #include <sys/ioctl.h>
 #include <linux/sockios.h>

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
	int Read(void *buf,const int buflen);
 
 ~CTcpClient();
};
 
int main()
{
  CTcpClient TcpClient;
 
  // 向服务器发起连接请求
  if (TcpClient.ConnectToServer("127.0.0.1",5005)==false)
  { printf("TcpClient.ConnectToServer(\"127.0.0.1\",5005) failed,exit...\n"); return -1; }
 
  //uint8_t strbuffer[4] {0x01,0x02,0x03,0x04};
  uint8_t strbuffer[2] {0x00,0x00};
 //memset(strbuffer,2,sizeof(strbuffer));
 uint8_t pending;
 uint8_t pending1;
uint8_t ret;
	while(1)
	{
	   // if (TcpClient.Send(strbuffer,4)<=0) break;


		//++strbuffer[7];

 //if (TcpClient.Recv(strbuffer,2)<=0) break;
ret=TcpClient.Recv(strbuffer,2);
 if (ret<=0) break;
sleep(0.01);
	for(int i{0};i<=1;++i)
		{
		 printf("%0x ",strbuffer[i]);
		}
printf("retvalue:  %d",ret);
	ioctl(TcpClient.m_sockfd,SIOCOUTQ,pending);
	ioctl(TcpClient.m_sockfd,FIONREAD,pending1);
printf("	SIOCOUTQ:%d  SIOCINQ:%d \n",pending,pending1);
 //if (TcpClient.Recv(strbuffer,1024)<=0) break;
TcpClient.Send(strbuffer,2);
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
