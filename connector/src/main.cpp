#include <iostream>
#include "Connector.h"
#include <unistd.h>
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "math.h"
#include "scout_msgs/ScoutStatus.h"
#include <iostream>
#include <random>
#include <memory>
#include <functional>
#include <tf/transform_broadcaster.h>
//#include "scout_msgs/ScoutBmsStatus"

//class Connector;
scout_msgs::ScoutStatus mystate;
geometry_msgs::Twist cmd;

auto start = std::chrono::steady_clock::now();
auto end = std::chrono::steady_clock::now();
 double deltat=0;
Connector ioport(500);
void recv(const geometry_msgs::Twist & cmdrecv);
void calc_odom(Connector &ioport, tf::TransformBroadcaster  &tfbr);
//void recv(const geometry_msgs::Twist & cmdrecv)

//{
//memcpy(&cmd,&cmdrecv,sizeof(cmdrecv));
//}

int main(int argc, char **argv)
{
  scout_msgs::ScoutStatus mystate;
 ros::init(argc, argv, "connector");
  //ros::init(argc, argv, "tf_pub");
  ros::NodeHandle connector_handle;


 tf::TransformBroadcaster  tfbr;
  // 创建一个Publisher，发布名为chatter的topic，消息类型为std_msgs::String
  ros::Publisher Msg_pub = connector_handle.advertise<scout_msgs::ScoutStatus> ("ScoutStatus", 10);
  ros::Rate loop_rate(50);
ioport.init();
 //auto f1 = std::bind(recv, _1,ioport);
//boost::bind(&recv,_1,&ioport);
  ros::Subscriber Msg_sub = connector_handle.subscribe("ScoutMotionCmd", 10, recv);
    while(1)
    {
    Msg_pub.publish(ioport.status_msg);
   calc_odom(ioport,tfbr);
	// 循环等待回调函数
   ros::spinOnce();
 //   printf("linear_vel:%lf m/s \n",ioport.scout_state.linear_velocity);
 //   printf("angular_velocity:%lf rad/s \n",ioport.scout_state.angular_velocity);
	// 按照循环频率延时
  loop_rate.sleep();
    }
}

void test_print_all_information(Connector &ioport)
{

    ioport.unpack_all();
    ioport.printall();
}

void test_cmd(char* cmd,double data,Connector &ioport)
{
	ioport.cmd_test();
}
void recv(const geometry_msgs::Twist & cmdrecv)

{
memcpy(&cmd,&cmdrecv,sizeof(cmdrecv));
ioport.SetMotionCommand(cmd.linear.y,0,cmd.angular.z, ioport.current_motion_cmd_.fault_clear_flag);
//printf("linear : %f,ang:%f\n",cmd.linear.y,cmd.angular.z);
//ioport.SetMotionCommand(0,0,0.5, ioport.current_motion_cmd_.fault_clear_flag);

}
void calc_odom(Connector &ioport, tf::TransformBroadcaster   &tfbr)
{
end = std::chrono::steady_clock::now();
 static double position_x_=0;
 static double position_y_=0;
 static double theta_=0;
 std::chrono::duration<double, std::micro> dt0 = end - start;
 double dt=0;
 dt=dt0.count()*0.000001; 
   double linear_speed_ =ioport.scout_state. linear_velocity;
    double angular_speed_ = ioport.scout_state. angular_velocity;

    double d_x = linear_speed_ * std::cos(theta_) * dt;
    double d_y = linear_speed_ * std::sin(theta_) * dt;
    double d_theta = angular_speed_ * dt;

    position_x_ += d_x;
    position_y_ += d_y;
    theta_ += d_theta;

     tf::Transform transform;
     transform.setOrigin(tf::Vector3(position_x_,position_y_,0.0));//设置平移变换
     tf::Quaternion q;
     q.setRPY(0,0,theta_);
     transform.setRotation(q);//设置角度变换
    tfbr.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"odom","base_footprint"));
   
     start = std::chrono::steady_clock::now();
}