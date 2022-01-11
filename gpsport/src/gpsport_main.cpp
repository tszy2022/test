#include <ros/ros.h>
#include "gps_driver.h"
#include "std_msgs/String.h"
//这次相对于上次的改进是加入了校正所收信息的代码，这样可以防止
//像之前收到aaaa这类乱码的可能性
int main(int argc, char **argv)
{
  ros::init(argc, argv, "gpsport");
  ros::NodeHandle node;

  gps_driver dvr(node);
  dvr.init();
  // dvr.Read_thread();
  while (ros::ok())
  {
    ros::spinOnce();
  }

  return 0;
}