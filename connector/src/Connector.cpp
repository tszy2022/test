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

    else
    {unsigned char strbuffer[13]={0x08,0x00,0x00,0x04,0x21,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
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
   printf("initialization completed. \n");
   return 1;
    }
}

void Connector::printall()//打印一次所有信息，调试专用函数
{

}
void Connector::copy_to_can_frame(can_frame *rx_frame, uint8_t *msg)
{
            ++msg;
        uint32_t iterim{0};
//          memcpy(&rx_frame->can_id,&iterim,1);
//          memcpy((&rx_frame->can_id)+1,&iterim,1);
//          memcpy((&rx_frame->can_id)+2,&iterim,1);
//          memcpy((&rx_frame->can_id)+3,&iterim,1);
            iterim=(
          static_cast<uint32_t>(msg[0]) |
          static_cast<uint32_t>(msg[2]) << 8 |
          static_cast<uint32_t>(msg[3]));
          printf("canID1: %0x finish \n",iterim);
        rx_frame->can_id=iterim;
        for(int count{0};count<=7;++count)
            {
                rx_frame->data[count]=msg[count+4];
            }
                    for(int count{0};count<=7;++count)
            {
              printf("%0x ",rx_frame->data[count]);
            }

switch (rx_frame->can_id)
{
case CAN_MSG_MOTION_COMMAND_ID:{ printf("getID!CAN_MSG_MOTION_COMMAND_ID\n");break;};
case CAN_MSG_LIGHT_COMMAND_ID:{ printf("getID!CAN_MSG_LIGHT_COMMAND_ID\n");break;};
case CAN_MSG_PARK_COMMAND_ID:{ printf("getID!CAN_MSG_PARK_COMMAND_ID\n");break;};
case CAN_MSG_SYSTEM_STATE_ID:{ printf("getID!CAN_MSG_SYSTEM_STATE_ID\n");break;};
case CAN_MSG_MOTION_STATE_ID:{ printf("getID!CAN_MSG_MOTION_STATE_ID\n");break;};
case CAN_MSG_LIGHT_STATE_ID:{ printf("getID!CAN_MSG_LIGHT_STATE_ID\n");break;};
case CAN_MSG_RC_STATE_ID:{ printf("getID!CAN_MSG_RC_STATE_ID\n");break;};
case CAN_MSG_ACTUATOR1_HS_STATE_ID:{ printf("getID!CAN_MSG_ACTUATOR1_HS_STATE_ID\n");break;};
case CAN_MSG_ACTUATOR2_HS_STATE_ID:{ printf("getID!CAN_MSG_ACTUATOR2_HS_STATE_ID\n");break;};
case CAN_MSG_ACTUATOR3_HS_STATE_ID:{ printf("getID!CAN_MSG_ACTUATOR3_HS_STATE_ID\n");break;};
case CAN_MSG_ACTUATOR4_HS_STATE_ID:{ printf("getID!CAN_MSG_ACTUATOR4_HS_STATE_ID\n");break;};
case CAN_MSG_ACTUATOR1_LS_STATE_ID:{ printf("getID!CAN_MSG_ACTUATOR1_LS_STATE_ID\n");break;};
case CAN_MSG_ACTUATOR2_LS_STATE_ID:{ printf("getID!CAN_MSG_ACTUATOR2_LS_STATE_ID\n");break;};
case CAN_MSG_ACTUATOR3_LS_STATE_ID:{ printf("getID!CAN_MSG_ACTUATOR3_LS_STATE_ID\n");break;};
case CAN_MSG_ACTUATOR4_LS_STATE_ID:{ printf("getID!CAN_MSG_ACTUATOR4_LS_STATE_ID\n");break;};
case CAN_MSG_ODOMETRY_ID:{ printf("getID!CAN_MSG_ODOMETRY_ID\n");break;};
case CAN_MSG_BMS_DATE_ID:{ printf("getID!CAN_MSG_BMS_DATE_ID\n");break;};
case CAN_MSG_BMS_STATUES_ID:{ printf("getID!CAN_MSG_BMS_STATUES_ID\n");break;};
case CAN_MSG_VERSION_QUERY_ID:{ printf("getID!CAN_MSG_VERSION_QUERY_ID\n");break;};
case CAN_MSG_PLATFORM_VERSION_ID:{ printf("getID!CAN_MSG_PLATFORM_VERSION_ID\n");break;};
case CAN_MSG_CTRL_MODE_SELECT_ID:{ printf("getID!CAN_MSG_CTRL_MODE_SELECT_ID\n");break;};
case CAN_MSG_STEER_NEUTRAL_RESET_ID:{ printf("getID!CAN_MSG_STEER_NEUTRAL_RESET_ID\n");break;};
case CAN_MSG_STEER_NEUTRAL_RESET_ACK_ID:{ printf("getID!CAN_MSG_STEER_NEUTRAL_RESET_ACK_ID\n");break;};
case CAN_MSG_STATE_RESET_ID:{ printf("getID!CAN_MSG_STATE_RESET_ID\n");break;};

}

}







void Connector::unpack_all() //打印一次所有信息，调试专用函数
{
     for (int count{ 0 }; count <= 100; ++count)
        {
           // printf("%d",count);
            if (Read(rec_buffer0,sizeof(rec_buffer0))<=0) break;
         //    memcpy(&mgs.raw, rec_buffer0+5,8 * sizeof(uint8_t));
      //   printf("%0x  \n",&(rec_buffer0[0]));
             copy_to_can_frame(can_frame_pt, &(rec_buffer0[0]));
            for (int j{ 0 }; j <=12; ++j)
                {
                    printf("%0x  ",rec_buffer0[j]);
                }
//                printf("****\n");
//                   for (int j{ 0 }; j <=7; ++j)
//                {
//                   printf("%0x ",mgs.raw[j]);
//                   //printf("%0x  ",mgs.cmd);
//
//                }
                printf("****\n");
        //       printf("%0x ",can_frame_pt->can_id);

                printf("end  \n");
                sleep(1);
        }

}



