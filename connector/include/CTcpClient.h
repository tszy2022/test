#ifndef CTCPCLIENT_H
#define CTCPCLIENT_H


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

    protected:

    private:
};

#endif // CTCPCLIENT_H
