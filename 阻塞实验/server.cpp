#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>
 #include <linux/sockios.h>
class CTcpServer
{
public:
  int m_listenfd;   // 服务端用于监听的socket
  int m_clientfd;   // 客户端连上来的socket
 
  CTcpServer();
 
  bool InitServer(int port);  // 初始化服务端
 
  bool Accept();  // 等待客户端的连接
 
  // 向对端发送报文
  int  Send(const void *buf,const int buflen);
  // 接收对端的报文
  int  Recv(void *buf,const int buflen);
 
  void CloseClient();    // 关闭客户端的socket
  void CloseListen();    // 关闭用于监听的socket
 
 ~CTcpServer();
};
 
CTcpServer TcpServer;
 
int main()
{
  // signal(SIGCHLD,SIG_IGN);  // 忽略子进程退出的信号，避免产生僵尸进程
 
  if (TcpServer.InitServer(5005)==false)
  { printf("服务端初始化失败，程序退出。\n"); return -1; }
 
  while (1)
  {
    if (TcpServer.Accept() == false) continue;
 
    if (fork()>0) { TcpServer.CloseClient(); continue; }  // 父进程回到while，继续Accept。
 
    // 子进程负责与客户端进行通信，直到客户端断开连接。
    TcpServer.CloseListen();
 
    printf("客户端已连接。\n");
 
    // 与客户端通信，接收客户端发过来的报文后，回复ok。
  uint8_t strbuffer[4] {0x01,0x02,0x03,0x04};
 uint8_t strbuffer0[200] {};
for(int i{0};i<=199;++i)
{
strbuffer0[i]=i;
}
 uint8_t strbuffer1[2] {0x00,0x00};
 //memset(strbuffer,2,sizeof(strbuffer0));
int j=0;
    while (1)
    {
      //memset(strbuffer,0,sizeof(strbuffer));
      //if (TcpServer.Recv(strbuffer,2)<=0) break;
	//for(int i{0};i<=1;++i)
	//	{
	//	 printf("%0x ",strbuffer[i]);
	//	}
	if (TcpServer.Send(strbuffer0,200)<=0) break;
	//if (TcpServer.Send(strbuffer0,1024)<=0) break;
	uint8_t pending;uint8_t pending1;uint8_t pending2;uint8_t pending3;
	ioctl(TcpServer.m_listenfd,SIOCOUTQ,pending);
	ioctl(TcpServer.m_listenfd,SIOCINQ,pending1);
	ioctl(TcpServer.m_clientfd,SIOCOUTQ,pending2);
	ioctl(TcpServer.m_clientfd,SIOCINQ,pending3);
printf("	SIOCOUTQ:%d  SIOCINQ:%d \n",pending,pending1);
printf("clientfd,SIOCOUTQ:%d**client,fdSIOCINQ:%d \n",pending2,pending3);
     	printf("\n");
	sleep(0.8);
     // if (TcpServer.Recv(strbuffer1,2)<=0) break;
	//for(int i{0};i<=1;++i)
	//	{
		 printf("%d ",++j);
	//	}
    }
 
    printf("客户端已断开连接。\n");
 
    return 0;  // 或者exit(0)，子进程退出。
  }
}
 
CTcpServer::CTcpServer()
{
  // 构造函数初始化socket
  m_listenfd=m_clientfd=0;
}
 
CTcpServer::~CTcpServer()
{
  if (m_listenfd!=0) close(m_listenfd);  // 析构函数关闭socket
  if (m_clientfd!=0) close(m_clientfd);  // 析构函数关闭socket
}
 
// 初始化服务端的socket，port为通信端口
bool CTcpServer::InitServer(int port)
{
  if (m_listenfd!=0) { close(m_listenfd); m_listenfd=0; }
 
  m_listenfd = socket(AF_INET,SOCK_STREAM,0);  // 创建服务端的socket
 
  // 把服务端用于通信的地址和端口绑定到socket上
  struct sockaddr_in servaddr;    // 服务端地址信息的数据结构
  memset(&servaddr,0,sizeof(servaddr));
  servaddr.sin_family = AF_INET;  // 协议族，在socket编程中只能是AF_INET
  servaddr.sin_addr.s_addr = htonl(INADDR_ANY);  // 本主机的任意ip地址
  servaddr.sin_port = htons(port);  // 绑定通信端口
  if (bind(m_listenfd,(struct sockaddr *)&servaddr,sizeof(servaddr)) != 0 )
  { close(m_listenfd); m_listenfd=0; return false; }
 
  // 把socket设置为监听模式
  if (listen(m_listenfd,5) != 0 ) { close(m_listenfd); m_listenfd=0; return false; }
 
  return true;
}
 
bool CTcpServer::Accept()
{
  if ( (m_clientfd=accept(m_listenfd,0,0)) <= 0) return false;
 
  return true;
}
 
int CTcpServer::Send(const void *buf,const int buflen)
{
  return send(m_clientfd,buf,buflen,0);
}
 
int CTcpServer::Recv(void *buf,const int buflen)
{
  return recv(m_clientfd,buf,buflen,0);
}
 
void CTcpServer::CloseClient()    // 关闭客户端的socket
{
  if (m_clientfd!=0) { close(m_clientfd); m_clientfd=0; }
}
 
void CTcpServer::CloseListen()    // 关闭用于监听的socket
{
  if (m_listenfd!=0) { close(m_listenfd); m_listenfd=0; }
}
