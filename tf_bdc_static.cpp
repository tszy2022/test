#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>
#include <stdio.h>
int main(int argc,char** argv)
{
    ros::init(argc, argv, "my_tf_broadcaster");
    ros::NodeHandle node;
    tf::TransformBroadcaster br;
     
     //根据乌龟当前的位姿。设置相对于世界坐标系的坐标变换
     tf::Transform transform;
     transform.setOrigin(tf::Vector3(0,0,0.0));//设置平移变换
     tf::Quaternion q;
     q.setRPY(0,0,0);
     transform.setRotation(q);//设置角度变换
 
     //发布坐标变换
    while(1)
    {
    // br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"odom","base_footprint"));

     //Odom-->base_footprint是需要scoutstate的
        br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"base_footprint","base_link"));
        br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"base_link","laser_link"));
        br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"base_link","base_laser"));
        br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"base_link","base_scan"));

    }
    return 0;
}