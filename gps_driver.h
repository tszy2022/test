#ifndef _GPS_DRIVER_H_
#define _GPS_DRIVER_H_

#include <string>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Int32.h>
#include <mutex>
#include <thread>
#include <aio.h>
#include "input.h"
#include "timer_base.h"

typedef struct
{
  /* myf added */
  uint32_t m_count  __attribute((aligned (8)));
  uint32_t rtk_time;
  uint8_t gps_status;
  uint8_t imu_status;
  uint8_t rtk_status;
  uint8_t fix_type;

  uint8_t satellites_used;
  uint8_t rtk_type;
  uint16_t year;//16位数据

  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t min;

  uint8_t sec;
  uint8_t week;
  uint8_t res8[2];//24位数据

  int32_t lat;//28
  int32_t lon;//32
  int32_t alt;
  //int32_t A_h;
  uint8_t A_h_alt;
  uint8_t A_h_lat;
  uint8_t A_h_lon;
  uint8_t A_h_void;
  int32_t x;
  int32_t y;
  uint32_t resu32[1];//8*4

  float hdop;
  float vx, vy, vz;
  float head;//朝向

  float ax, ay, az;
  float thex, they, thez;
  float wx, wy, wz;
  float resf[4];//13*4

  //我加的

  double high_lat;
  double high_lon;
  double high_alt;
} dINS_DATA;

class gps_driver
{
public:
  int m_sockfd {0};
  int baud_rate;
  int wait_time = 10;
  int recsize=500;
  int line_num = 0;
  int control_period_ms{10};
  uint8_t rec_buffer0[13]{};
  uint8_t rec_buffer[500]{};
  uint8_t snd_buffer[13]{};
  Timer read_tm;
  Timer testcmd;
  Timer control_tm;
  dINS_DATA rec_data;
  aiocb aior;
  aiocb aiow;
  std::mutex read_mutex;
  std::mutex motion_cmd_mutex_;
  std::thread read_thread;
  std::thread control_thread;
  ros::NodeHandle nh;
  ros::Publisher Msg_pub ;
int bias{0};
int length{0};
int check_head=0;
int check_tail=1000;
  gps_driver(ros::NodeHandle node);

  ~gps_driver();
  void init();
  bool poll();
  int start_read_thread();
  void Read_thread();
  bool ConnectToServer(const char *serverip, const int port);
  int check_header(const uint8_t *rec_buffer,int i);
  void copy_to_single_frame(dINS_DATA *rec_data, const uint8_t *rec_buffer);
  //void DecodesingleFrame(can_frame_pt, agx_msg_pt);
 void convert_data_once();
 void broadcastcor();
};

#endif
