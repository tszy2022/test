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
#include <assert.h>
#include <errno.h>
#include <cstdlib>
#include <thread>


uint8_t strbuffer[8192] {0x0a,0x02,0x08,0x03};
uint8_t strbuffer0[8192] {0x0a,0x02,0x08,0x03};

// TCP客户端类
class CTcpClient
{
public:
    int m_sockfd;
    aiocb aior;
    aiocb aiow;
    CTcpClient();

    // 向服务器发起连接，serverip-服务端ip，port通信端口
    bool ConnectToServer(const char *serverip,const int port);
    // 向对端发送报文
    int  Send(const void *buf,const int buflen);
    // 接收对端的报文
    int  Recv(void *buf,const int buflen);
    int  aRecv(volatile void *buf,const int buflen);
    int aSend(volatile void *buf,const int buflen);
    int Read(void *buf,const int buflen);
    int read_thread();
    ~CTcpClient();
};
int read_thread(int fd);
int write_thread(int fd);
int main()
{
    CTcpClient TcpClient;
    int a;
    for(int i{0}; i<=8191; ++i)
    {
        strbuffer0[i]=i;
    }
    // 向服务器发起连接请求
    if (TcpClient.ConnectToServer("127.0.0.1",5005)==false)
    {
        printf("TcpClient.ConnectToServer(\"127.0.0.1\",5005) failed,exit...\n");
        return -1;
    }

    //std::thread(read_thread, std::ref(TcpClient));
    printf("1");
    std::thread t1(read_thread,TcpClient.m_sockfd);
    std::thread t2(write_thread,TcpClient.m_sockfd);
    while(1)
    {
//        int couter=0;
//
//        ret=TcpClient.aRecv(strbuffer,8192);
//        printf("ret: %0x   \n",ret);
//
//        for(int i{0}; i<=10; ++i)
//        {
//            printf("%0x ",strbuffer[i]);
//        }
        //TcpClient.aSend(strbuffer0,8192);
       // printf("1\n");
 //       TcpClient.Send(strbuffer0,8192);
       // sleep(1);
    }

}

CTcpClient::CTcpClient()
{
    m_sockfd=0;
}

CTcpClient::~CTcpClient()
{
    if (m_sockfd!=0) close(m_sockfd);
}

bool CTcpClient::ConnectToServer(const char *serverip,const int port)
{
    m_sockfd = socket(AF_INET,SOCK_STREAM,0);

    struct hostent* h;
    if ( (h=gethostbyname(serverip))==0 )
    {
        close(m_sockfd);
        m_sockfd=0;
        return false;
    }

    struct sockaddr_in servaddr;
    memset(&servaddr,0,sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(port);
    memcpy(&servaddr.sin_addr,h->h_addr,h->h_length);

    if (connect(m_sockfd,(struct sockaddr *)&servaddr,sizeof(servaddr))!=0)
    {
        close(m_sockfd);
        m_sockfd=0;
        return false;
    }
    memset(&aior, 0, sizeof(aior));
    memset(&aiow, 0, sizeof(aiow));
    aior.aio_fildes=m_sockfd;
    aiow.aio_fildes=m_sockfd;
    aior.aio_offset=0;
    aiow.aio_offset=0;
    return true;
}

int CTcpClient::Send(const void *buf,const int buflen)
{
    return send(m_sockfd,buf,buflen,0);
}
int CTcpClient::aSend(volatile void *buf,const int buflen)
{
    int ret,a=0;


    aiow.aio_buf=buf;
    aiow.aio_fildes=m_sockfd;
    aiow.aio_nbytes=buflen;
    ret=aio_write(&aiow);

    if (ret<0)
    {
        perror("aio_write");
        exit(1);
    }
    while(aio_error(&aiow)==EINPROGRESS)
    {
      //  printf("client send 第 %d 次 \n",++a);
    }

    return ret;
}
int CTcpClient::Recv(void *buf,const int buflen)
{
    return recv(m_sockfd,buf,buflen,0);
}
int CTcpClient::Read(void *buf,const int buflen)
{
    return read(m_sockfd,buf,buflen);
}
int CTcpClient::aRecv(volatile void *buf,const int buflen)
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
        printf("client recv 第 %d 次 \n",++a);
    }

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
    //printf("1");
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
            //   printf("client recv 第 %d 次 \n",++a);
            //     ++a;
        }

        for(int i{0}; i<=10; ++i)
        {
            printf("%0x ",strbuffer[i]);
        }
       // sleep(1);
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
        //printf("%d \n",ret);
        if (ret<0)
        {
            perror("aio_write");
            exit(1);
        }
        while(aio_error(&aiow)==EINPROGRESS)
        {
           // printf("client send 第 %d 次 \n",++a);
            //  ++a;
        }
      //  sleep(1);

        printf("\n");

    }
}
