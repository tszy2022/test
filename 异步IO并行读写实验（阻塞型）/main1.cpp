#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <linux/sockios.h>
#include <aio.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <assert.h>
#include <cstdlib>
#include  <thread>

uint8_t strbuffer[8192] {};
uint8_t strbuffer0[8192] {1,3,7,6,7,7,3,6};
int write_thread(int fd);
int read_thread(int fd);
class CTcpServer
{
public:
    int m_listenfd;   // 服务端用于监听的socket
    int m_clientfd;   // 客户端连上来的socket
    aiocb aiow;
    aiocb aior;
    CTcpServer();

    bool InitServer(int port);  // 初始化服务端

    bool Accept();  // 等待客户端的连接

    // 向对端发送报文
    int  Send(const void *buf,const int buflen);
    int  aSend(volatile void *buf,const int buflen);
    int  aRecv(volatile void *buf,const int buflen);
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
    {
        printf("服务端初始化失败，程序退出。\n");
        return -1;
    }

    while (1)
    {
        if (TcpServer.Accept() == false) continue;

        if (fork()>0)
        {
            TcpServer.CloseClient();    // 父进程回到while，继续Accept。
            continue;
        }

        // 子进程负责与客户端进行通信，直到客户端断开连接。
        TcpServer.CloseListen();

        printf("客户端已连接。\n");
        //printf("TcpServer.m_listenfd:%d  TcpServer.m_clientfd:%d \n",TcpServer.m_listenfd,TcpServer.m_clientfd);
        // 与客户端通信，接收客户端发过来的报文后，回复ok。
        uint8_t strbuffer3[8192] {};
        for(int i{0}; i<=8191; ++i)
        {
            strbuffer3[i]=i;
        }
//memset(strbuffer,2,sizeof(strbuffer0));
//       int j=0;
    std::thread t1(read_thread,dup(TcpServer.m_clientfd));
    std::thread t2(write_thread,dup(TcpServer.m_clientfd));
        while (1)
        {
            //memset(strbuffer,0,sizeof(strbuffer));
//            if (TcpServer.Recv(strbuffer0,2)<=0) break;
//            for(int i{0};i<=2;++i)
//            	{
//            	 printf("%0x ",strbuffer0[i]);
//            	}
            //if (TcpServer.Send(strbuffer0,200)<=0) break;
//            if (TcpServer.aSend(strbuffer3,8192)<0) break;
            //if (TcpServer.aRecv(strbuffer0,2)<0) break;
            //sleep(1);

//	uint8_t pending;uint8_t pending1;uint8_t pending2;uint8_t pending3;
//	ioctl(TcpServer.m_listenfd,SIOCOUTQ,pending);
//	ioctl(TcpServer.m_listenfd,SIOCINQ,pending1);
//	ioctl(TcpServer.m_clientfd,SIOCOUTQ,pending2);
//	ioctl(TcpServer.m_clientfd,SIOCINQ,pending3);
//printf("	SIOCOUTQ:%d  SIOCINQ:%d \n",pending,pending1);
//printf("clientfd,SIOCOUTQ:%d**client,fdSIOCINQ:%d \n",pending2,pending3);
//     	printf("\n");
            //  if (TcpServer.Recv(strbuffer1,2)<=0) break;
            //for(int i{0};i<=1;++i)
            //	{
           // printf("%d \n",++j);
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
    if (m_listenfd!=0)
    {
        close(m_listenfd);
        m_listenfd=0;
    }

    m_listenfd = socket(AF_INET,SOCK_STREAM,0);  // 创建服务端的socket

    // 把服务端用于通信的地址和端口绑定到socket上
    struct sockaddr_in servaddr;    // 服务端地址信息的数据结构
    memset(&servaddr,0,sizeof(servaddr));
    servaddr.sin_family = AF_INET;  // 协议族，在socket编程中只能是AF_INET
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);  // 本主机的任意ip地址
    servaddr.sin_port = htons(port);  // 绑定通信端口
    if (bind(m_listenfd,(struct sockaddr *)&servaddr,sizeof(servaddr)) != 0 )
    {
        close(m_listenfd);
        m_listenfd=0;
        return false;
    }

    // 把socket设置为监听模式
    if (listen(m_listenfd,5) != 0 )
    {
        close(m_listenfd);
        m_listenfd=0;
        return false;
    }
    //bzero(&aiow,sizeof(m_clientfd));
    memset(&aiow, 0, sizeof(aiow));
    memset(&aior, 0, sizeof(aior));

    aior.aio_nbytes=4;
    aior.aio_offset=0;
    aiow.aio_nbytes=4;
    aiow.aio_offset=0;
    return true;
}

bool CTcpServer::Accept()
{
    if ( (m_clientfd=accept(m_listenfd,0,0)) <= 0) return false;
    aiow.aio_fildes=m_clientfd;
    aior.aio_fildes=m_clientfd;
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
    if (m_clientfd!=0)
    {
        close(m_clientfd);
        m_clientfd=0;
    }
}

void CTcpServer::CloseListen()    // 关闭用于监听的socket
{
    if (m_listenfd!=0)
    {
        close(m_listenfd);
        m_listenfd=0;
    }
}
int CTcpServer::aSend(volatile void *buf,const int buflen)
{
    int a=1;
    int fd,ret,couter;
    uint8_t strbuffer0[8192] {};
    aiow.aio_buf=buf;
    aiow.aio_nbytes=buflen;
    ret=aio_write(&aiow);
    //阻塞版本：
    while(aio_error(&aiow)==EINPROGRESS)
    {
    //    printf("server write 第 %d 次 \n",++a);
        //printf("a %d\n",a);是电驱动
        ++a;
    }
    //非阻塞版本

    return aio_error(&aiow);
}
int CTcpServer::aRecv(volatile void *buf,const int buflen)
{
    int ret,a=0;

    aior.aio_buf=buf;

    aior.aio_nbytes=buflen;
    ret=aio_read(&aior);

    if (ret<0)
    {
        perror("aio_read");
        exit(1);
    }
    while(aio_error(&aior)==EINPROGRESS)
    {
      //  printf("serv recv 第 %d 次 \n",++a);
    }
//        for(int i{0}; i<=10; ++i)
//        {
//            printf("%0x ",strbuffer0[i]);
//        }
//        printf("\n");
    return ret;
}

int read_thread(int fd)
{
    int ret,a=0;
    aiocb aior;
    memset(&aior, 0, sizeof(aior));
    aior.aio_fildes=fd;
    aior.aio_offset=0;
    aior.aio_buf=strbuffer;

    aior.aio_nbytes=8192;
   // printf("1");
    while(1)
    {
        a=0;
        ret=aio_read(&aior);
        //printf("%d \n",ret);
        if (ret<0)
        {
            perror("aio_read");
            exit(1);
        }
        while(aio_error(&aior)==EINPROGRESS)
        {
          //  printf("client recv 第 %d 次 \n",++a);
               // ++a;
        }

        for(int i{0}; i<=10; ++i)
        {
            printf("%0x ",strbuffer[i]);
        }
        printf("\n");

    }
}

int write_thread(int fd)
{
    int ret,a=0;
    aiocb aiow;
    memset(&aiow, 0, sizeof(aiow));
    aiow.aio_fildes=fd;
    aiow.aio_offset=0;
    aiow.aio_buf=strbuffer0;

    aiow.aio_nbytes=8192;
    //printf("1");
    while(1)
    {
        a=0;
        ret=aio_write(&aiow);
        printf("%d \n",ret);
        if (ret<0)
        {
            perror("aio_write");
            exit(1);
        }
        while(aio_error(&aiow)==EINPROGRESS)
        {
          //  printf("client send 第 %d 次 \n",++a);
               // ++a;
        }

        printf("\n");

    }
}

