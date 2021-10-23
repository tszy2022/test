#include <ros/ros.h>
#include "message_filters/subscriber.h"
#include "sensor_msgs/LaserScan.h"
#include <math.h>
#include "std_msgs/String.h"
#include "Controller.h"
#include "commute/plan_path.h"
#include <stdio.h>
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "control_cmd");
    ros::NodeHandle control_cmd;
    printf("control_cmd");
    ros::Rate loop_rate(60);

    Controller my_control;
    ros::ServiceClient client = control_cmd.serviceClient<commute::plan_path>("plan_path");

    my_control.nh = control_cmd;
    my_control.client = client;
    my_control.init();
    my_control.require_service();
    printf("finish service \n");
    while (1)

    {
       
     //   my_control.require_tf(); // on another thread
        my_control.calc_points();
        my_control.send_cmd();
        loop_rate.sleep();
    }
    return 0;
}