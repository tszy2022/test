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
  int m_listenfd;   // ��������ڼ�����socket
  int m_clientfd;   // �ͻ�����������socket
 
  CTcpServer();
 
  bool InitServer(int port);  // ��ʼ�������
 
  bool Accept();  // �ȴ��ͻ��˵�����
 
  // ��Զ˷��ͱ���
  int  Send(const void *buf,const int buflen);
  // ���նԶ˵ı���
  int  Recv(void *buf,const int buflen);
 
  void CloseClient();    // �رտͻ��˵�socket
  void CloseListen();    // �ر����ڼ�����socket
 
 ~CTcpServer();
};
 
CTcpServer TcpServer;
 
int main()
{
  // signal(SIGCHLD,SIG_IGN);  // �����ӽ����˳����źţ����������ʬ����
 
  if (TcpServer.InitServer(5005)==false)
  { printf("����˳�ʼ��ʧ�ܣ������˳���\n"); return -1; }
 
  while (1)
  {
    if (TcpServer.Accept() == false) continue;
 
    if (fork()>0) { TcpServer.CloseClient(); continue; }  // �����̻ص�while������Accept��
 
    // �ӽ��̸�����ͻ��˽���ͨ�ţ�ֱ���ͻ��˶Ͽ����ӡ�
    TcpServer.CloseListen();
 
    printf("�ͻ��������ӡ�\n");
 
    // ��ͻ���ͨ�ţ����տͻ��˷������ı��ĺ󣬻ظ�ok��
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
 
    printf("�ͻ����ѶϿ����ӡ�\n");
 
    return 0;  // ����exit(0)���ӽ����˳���
  }
}
 
CTcpServer::CTcpServer()
{
  // ���캯����ʼ��socket
  m_listenfd=m_clientfd=0;
}
 
CTcpServer::~CTcpServer()
{
  if (m_listenfd!=0) close(m_listenfd);  // ���������ر�socket
  if (m_clientfd!=0) close(m_clientfd);  // ���������ر�socket
}
 
// ��ʼ������˵�socket��portΪͨ�Ŷ˿�
bool CTcpServer::InitServer(int port)
{
  if (m_listenfd!=0) { close(m_listenfd); m_listenfd=0; }
 
  m_listenfd = socket(AF_INET,SOCK_STREAM,0);  // ��������˵�socket
 
  // �ѷ��������ͨ�ŵĵ�ַ�Ͷ˿ڰ󶨵�socket��
  struct sockaddr_in servaddr;    // ����˵�ַ��Ϣ�����ݽṹ
  memset(&servaddr,0,sizeof(servaddr));
  servaddr.sin_family = AF_INET;  // Э���壬��socket�����ֻ����AF_INET
  servaddr.sin_addr.s_addr = htonl(INADDR_ANY);  // ������������ip��ַ
  servaddr.sin_port = htons(port);  // ��ͨ�Ŷ˿�
  if (bind(m_listenfd,(struct sockaddr *)&servaddr,sizeof(servaddr)) != 0 )
  { close(m_listenfd); m_listenfd=0; return false; }
 
  // ��socket����Ϊ����ģʽ
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
 
void CTcpServer::CloseClient()    // �رտͻ��˵�socket
{
  if (m_clientfd!=0) { close(m_clientfd); m_clientfd=0; }
}
 
void CTcpServer::CloseListen()    // �ر����ڼ�����socket
{
  if (m_listenfd!=0) { close(m_listenfd); m_listenfd=0; }
}
