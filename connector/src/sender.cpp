#include <iostream>
#include "Connector.h"
#include <unistd.h>
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "math.h"
#include "scout_msgs/ScoutStatus.h"
  scout_msgs::ScoutStatus mystate;
  geometry_msgs::Twist cmd;

void recv(const scout_msgs::ScoutStatus &status)

{
memcpy(&mystate,&status,sizeof(status));

}
int main(int argc, char **argv)
{

  cmd.linear.y=1;
  cmd.angular.z=0.2;

 ros::init(argc, argv, "control_console");
  ros::NodeHandle controlhd;

  ros::Publisher Msg_pub = controlhd.advertise< geometry_msgs::Twist> ("ScoutMotionCmd", 10);
 ros::Subscriber Msg_sub=controlhd.subscribe ("ScoutStatus", 10,recv);

  ros::Rate loop_rate(10);

    while(1)
    {
    Msg_pub.publish(cmd);

    ros::spinOnce();

    loop_rate.sleep();
    }
}
