#include <iostream>
#include "Connector.h"
#include <unistd.h>
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "math.h"
#include "scout_msgs/ScoutStatus.h"
#include <stdio.h>
#include <termio.h>
geometry_msgs::Twist cmd;
int main(int argc, char **argv)
{
termios tms_old,tms_new;
tcgetattr(0,&tms_old);
tms_new=tms_old;

tms_new.c_lflag=~(ICANON|ECHO);
tcsetattr(0,TCSANOW,&tms_new);




 ros::init(argc, argv, "control_console");
  ros::NodeHandle key_control;

  ros::Publisher Msg_pub = key_control.advertise< geometry_msgs::Twist> ("ScoutMotionCmd", 10);
// ros::Subscriber Msg_sub=key_control.subscribe ("ScoutStatus", 10,recv);

  ros::Rate loop_rate(50);
double count=0.1;double delta=0.1;
double count1=0;
    while(1)
    {
    unsigned char ch=getchar();
switch (ch)
{
    case 0x77: //w
    {
     cmd.linear.y+= count;
  break;
    }
    case 0x73://s
    {
     cmd.linear.y-= count;
break;
    }
    case 0x61: //a
    {
cmd.angular.z-=count;
break;
    }
    case 0x64://d
    {
cmd.angular.z+=count;
break;
    }
        case 0x6f://o
    {
cmd.angular.z=0;
cmd.linear.y=0;

break;
    }
    default:
    cmd.angular.z=0;
cmd.linear.y=0;
        break;
}



    Msg_pub.publish(cmd);

    ros::spinOnce();

    loop_rate.sleep();
    printf("linear : %f,ang:%f\n",cmd.linear.y,cmd.angular.z);
    }
}