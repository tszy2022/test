#include "gpsport/gps_driver.h"
#include <unistd.h>
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sensor_msgs/NavSatFix.h>
gps_driver::gps_driver(ros::NodeHandle node)
{

  double packet_rate; // packet frequency (Hz)
  nh = node;
  packet_rate = 50; //20000/24

  printf("init gps_driver \n");
}

gps_driver::~gps_driver()
{
}

bool gps_driver::poll(void)
{ // Allocate a new shared pointer for zero-copy sharing with other nodelets.
}
void gps_driver::init()

{
  printf("init m_sock_var :%d \n", m_sockfd);
  gps_driver::ConnectToServer("192.168.1.253", 1030); //这里改成自己要的
  ros::Publisher Msg_pub = nh.advertise<sensor_msgs::NavSatFix>("fix", 20);
  gps_driver::start_read_thread();
}

bool gps_driver::ConnectToServer(const char *serverip, const int port)
{
  printf("port is %d \n", port);
  m_sockfd = socket(AF_INET, SOCK_STREAM, 0); // 创建客户端的socket

  struct hostent *h; // ip地址信息的数据结构
  if ((h = gethostbyname(serverip)) == 0)
  {
    close(m_sockfd);
    m_sockfd = 0;
    ROS_WARN("get ip: %s failed", serverip);
    return false;
  }

  // 把服务器的地址和端口转换为数据结构
  struct sockaddr_in servaddr;
  memset(&servaddr, 0, sizeof(servaddr));
  servaddr.sin_family = AF_INET;
  servaddr.sin_port = htons(port);
  memcpy(&servaddr.sin_addr, h->h_addr, h->h_length);

  // 向服务器发起连接请求
  if (connect(m_sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr)) != 0)
  {
    perror("Error: ");
    close(m_sockfd);

    m_sockfd = 0;
    ROS_WARN("setup the sockect failed");
    return false;
  }
  ROS_WARN("setup the sockect succeed");
  return true;
}
int gps_driver::start_read_thread()
{
  if (m_sockfd == 0)
  {
    ROS_WARN("can not set up read thread");
    return 0;
  }
  else
  {
    read_thread = std::thread(&gps_driver::Read_thread, this);
    return true;
  }
}
void gps_driver::Read_thread()
{
  int ret, a = 0;
  memset(&aior, 0, sizeof(aior));

  if (m_sockfd == 0)
  {
    ROS_WARN("the socket has not gotten connect");
  }

  aior.aio_fildes = m_sockfd;
  aior.aio_offset = 0;
  aior.aio_buf = rec_buffer;
  aior.aio_nbytes = recsize;

  while (ros::ok())
  {
    read_tm.reset();
    a = 0;
    read_mutex.lock();
    ret = aio_read(&aior);
    read_mutex.unlock();
    if (ret < 0)
    {
      ROS_ERROR("aio_read failed");
      exit(1);
    }
    //这里要考虑一下了
   //for (int i = 0; i <= 600; i++)
    // {
   //  printf("%x ", rec_buffer[i]);
  //  }
    printf("one frame \n");
    for (int i{0}; i <= line_num; ++i)
    {
      bias = check_header(rec_buffer, i);
      copy_to_single_frame(&rec_data, &(rec_buffer[bias +6]));

      convert_data_once();
    }
    length = 0;
    broadcastcor();
    read_tm.sleep_until_ms(10);
  }
}

void gps_driver::copy_to_single_frame(dINS_DATA *rec_data, const uint8_t *rec_buffer)
{


  rec_data->m_count = (static_cast<uint32_t>(rec_buffer[0]) |
                       static_cast<uint32_t>(rec_buffer[1]) << 8 |
                       static_cast<uint32_t>(rec_buffer[2]) << 16 |
                       static_cast<uint32_t>(rec_buffer[3]) << 24);
  rec_data->rtk_time = (static_cast<uint32_t>(rec_buffer[4]) |
                        static_cast<uint32_t>(rec_buffer[5]) << 8 |
                        static_cast<uint32_t>(rec_buffer[6]) << 16 |
                        static_cast<uint32_t>(rec_buffer[7]) << 24);
  rec_data->gps_status = rec_buffer[8];
  rec_data->imu_status = rec_buffer[9];
  rec_data->rtk_status = rec_buffer[10];
  rec_data->fix_type = rec_buffer[11];
  rec_data->satellites_used = rec_buffer[12];
  rec_data->rtk_type = rec_buffer[13];

  rec_data->year = (static_cast<uint16_t>(rec_buffer[14]) |
                    static_cast<uint16_t>(rec_buffer[15]) << 8);

  rec_data->month = rec_buffer[16];

  rec_data->day = rec_buffer[17];
  rec_data->hour = rec_buffer[18];
  rec_data->min = rec_buffer[19];
  rec_data->sec = rec_buffer[20];
printf("%d year %d month %d day %d hour %d min %d sec \n",rec_data->year,rec_data->month,rec_data->day,rec_data->hour,rec_data->min,rec_data->sec);
  rec_data->week = rec_buffer[21];


  rec_data->lat = (static_cast<uint32_t>(rec_buffer[24]) |
                   static_cast<uint32_t>(rec_buffer[25]) << 8 |
                   static_cast<uint32_t>(rec_buffer[26]) << 16 |
                   static_cast<uint32_t>(rec_buffer[27]) << 24);

  rec_data->lon = (static_cast<uint32_t>(rec_buffer[28]) |
                   static_cast<uint32_t>(rec_buffer[29]) << 8 |
                   static_cast<uint32_t>(rec_buffer[30]) << 16 |
                   static_cast<uint32_t>(rec_buffer[31]) << 24);
//printf("%d lat %d lon \n",rec_data->lat ,rec_data->lon);
  rec_data->alt = (static_cast<uint32_t>(rec_buffer[32]) |
                   static_cast<uint32_t>(rec_buffer[33]) << 8 |
                   static_cast<uint32_t>(rec_buffer[34]) << 16 |
                   static_cast<uint32_t>(rec_buffer[35]) << 24);
  rec_data->A_h_alt = rec_buffer[36];
  rec_data->A_h_lat = rec_buffer[37];
  rec_data->A_h_lon = rec_buffer[38];
  rec_data->A_h_void = rec_buffer[39];
  rec_data->x = (static_cast<uint32_t>(rec_buffer[40]) |
                 static_cast<uint32_t>(rec_buffer[41]) << 8 |
                 static_cast<uint32_t>(rec_buffer[42]) << 16 |
                 static_cast<uint32_t>(rec_buffer[43]) << 24);
  rec_data->y = (static_cast<uint32_t>(rec_buffer[44]) |
                 static_cast<uint32_t>(rec_buffer[45]) << 8 |
                 static_cast<uint32_t>(rec_buffer[46]) << 16 |
                 static_cast<uint32_t>(rec_buffer[47]) << 24);

  rec_data->hdop = (static_cast<uint32_t>(rec_buffer[56]) |
                    static_cast<uint32_t>(rec_buffer[57]) << 8 |
                    static_cast<uint32_t>(rec_buffer[58]) << 16 |
                    static_cast<uint32_t>(rec_buffer[59]) << 24);
  rec_data->vx = (static_cast<uint32_t>(rec_buffer[60]) |
                  static_cast<uint32_t>(rec_buffer[61]) << 8 |
                  static_cast<uint32_t>(rec_buffer[62]) << 16 |
                  static_cast<uint32_t>(rec_buffer[63]) << 24);
  rec_data->vy = (static_cast<uint32_t>(rec_buffer[64]) |
                  static_cast<uint32_t>(rec_buffer[65]) << 8 |
                  static_cast<uint32_t>(rec_buffer[66]) << 16 |
                  static_cast<uint32_t>(rec_buffer[67]) << 24);
  rec_data->vz = (static_cast<uint32_t>(rec_buffer[68]) |
                  static_cast<uint32_t>(rec_buffer[69]) << 8 |
                  static_cast<uint32_t>(rec_buffer[70]) << 16 |
                  static_cast<uint32_t>(rec_buffer[71]) << 24);
  rec_data->head = (static_cast<uint32_t>(rec_buffer[72]) |
                    static_cast<uint32_t>(rec_buffer[73]) << 8 |
                    static_cast<uint32_t>(rec_buffer[74]) << 16 |
                    static_cast<uint32_t>(rec_buffer[75]) << 24);
  rec_data->ax = (static_cast<uint32_t>(rec_buffer[76]) |
                  static_cast<uint32_t>(rec_buffer[77]) << 8 |
                  static_cast<uint32_t>(rec_buffer[78]) << 16 |
                  static_cast<uint32_t>(rec_buffer[79]) << 24);
  rec_data->ay = (static_cast<uint32_t>(rec_buffer[80]) |
                  static_cast<uint32_t>(rec_buffer[81]) << 8 |
                  static_cast<uint32_t>(rec_buffer[82]) << 16 |
                  static_cast<uint32_t>(rec_buffer[83]) << 24);
  rec_data->az = (static_cast<uint32_t>(rec_buffer[84]) |
                  static_cast<uint32_t>(rec_buffer[85]) << 8 |
                  static_cast<uint32_t>(rec_buffer[86]) << 16 |
                  static_cast<uint32_t>(rec_buffer[87]) << 24);
//printf("%.10lf ax %.10lf ay %.10lf az \n", rec_data->ax ,rec_data->ay,rec_data->az);
  rec_data->thex = (static_cast<uint32_t>(rec_buffer[88]) |
                    static_cast<uint32_t>(rec_buffer[89]) << 8 |
                    static_cast<uint32_t>(rec_buffer[90]) << 16 |
                    static_cast<uint32_t>(rec_buffer[91]) << 24);
  rec_data->they = (static_cast<uint32_t>(rec_buffer[92]) |
                    static_cast<uint32_t>(rec_buffer[93]) << 8 |
                    static_cast<uint32_t>(rec_buffer[94]) << 16 |
                    static_cast<uint32_t>(rec_buffer[95]) << 24);
  rec_data->thez = (static_cast<uint32_t>(rec_buffer[96]) |
                    static_cast<uint32_t>(rec_buffer[97]) << 8 |
                    static_cast<uint32_t>(rec_buffer[98]) << 16 |
                    static_cast<uint32_t>(rec_buffer[99]) << 24);
  rec_data->wx = (static_cast<uint32_t>(rec_buffer[100]) |	
                  static_cast<uint32_t>(rec_buffer[101]) << 8 |
                  static_cast<uint32_t>(rec_buffer[102]) << 16 |
                  static_cast<uint32_t>(rec_buffer[103]) << 24);

  rec_data->wy = (static_cast<uint32_t>(rec_buffer[104]) |
                  static_cast<uint32_t>(rec_buffer[105]) << 8 |
                  static_cast<uint32_t>(rec_buffer[106]) << 16 |
                  static_cast<uint32_t>(rec_buffer[107]) << 24);
  rec_data->wz = (static_cast<uint32_t>(rec_buffer[108]) |
                  static_cast<uint32_t>(rec_buffer[109]) << 8 |
                  static_cast<uint32_t>(rec_buffer[110]) << 16 |
                  static_cast<uint32_t>(rec_buffer[111]) << 24);
}
int gps_driver::check_header(const uint8_t *rec_buffer, int i)
{
  int bs{0};
  if (i != 0)
  {
    for (int j = 0; j <= check_head + check_tail; j++)
    {
      if (rec_buffer[j - check_head + length] == 0xB5)
      {
        if (rec_buffer[j - check_head + length + 1] == 0x62)
        {
          if (rec_buffer[j - check_head + length + 2] == 0x01)
          {
            if (rec_buffer[j - check_head + length + 3] == 0xF2)
            {
              bs = j - check_head + length;
              length = bs+132;
//printf("%d \n ",bs);
              return bs;
            }
          }
        }
      }
    }
  }
  else
  {
    for (int j = 0; j <= check_head + check_tail; j++)
    {
      if (rec_buffer[j] == 0xB5)
      {
        if (rec_buffer[j + 1] == 0x62)
        {
          if (rec_buffer[j + 2] == 0x01)
          {
            if (rec_buffer[j + 3] == 0xF2)
            {
              bs = j;
//printf("%d \n ",bs);
                           length = bs+132; 
              return bs;
            }
          }
        }
      }
    }
  }
}
void gps_driver::convert_data_once()
{
  rec_data.high_lat = static_cast<double>(rec_data.lat / 1000) / 10000 + static_cast<double>(rec_data.A_h_lat ) / 1000000;
  //分别为10^-7与10^-15,有点担心精度问题
  rec_data.high_lon = static_cast<double>(rec_data.lon / 1000) / 10000 + static_cast<double>(rec_data.A_h_lon ) / 1000000;
  rec_data.high_alt = static_cast<double>(rec_data.alt) + static_cast<double>(rec_data.A_h_alt)/1000;
printf("%.10lf lat %.10lf lon %.10lf high \n", rec_data.high_lat ,rec_data.high_lon,rec_data.high_alt);
}

void gps_driver::broadcastcor()
{
  

}