#include "Controller.h"
#include <stdio.h>
#include "ros/ros.h"
#include <tf2_ros/transform_listener.h>
#include <mutex>
#include <thread>
#include <tf2/LinearMath/Matrix3x3.h>
#include <math.h>
#include "geometry_msgs/Twist.h"
#include "commute/plan_path.h"

#define PI 3.1415926535
Controller::Controller()
{
}
Controller::~Controller()
{
}
void Controller::start_Read_thread()
{
    ros::Rate loop_rate(20);
    tf2_ros::TransformListener listener(buffer);
    ros::Duration(2).sleep();

    while (1)
    {

        loop_rate.sleep();
    }
}
void Controller::find_min(double x_now, double y_now)
{

    double dx = 0;
    double dy = 0;
    int flag = 0;
    double distance[srv.response.max_num]{};
    int cntx = 0;
    // printf("x_now is : %f   y_now is %f \n",x_now,y_now);
       // sleep(1);
    for (int i = 0; i < search_length && index + i < srv.response.max_num; i++)
    {

        dx = srv.response.posx[index + i] - x_now;
        dy = srv.response.posy[index + i] - y_now;
        distance[i] = sqrt(dx * dx + dy * dy);
        
        cntx++;
    }
    for (int i = 0; i < search_length && i < srv.response.max_num; i++)
    {
        for (int j = i; j < search_length && j < srv.response.max_num; j++)
        {
           // printf("distance i  :%f distance j :%f  x_now:%f  y_now :%f \n",distance[i],distance[j],x_now,y_now);
            if (distance[i] > distance[j])
            {
                break;
            }
            if (j+1==search_length || j+1==srv.response.max_num)
            {
             //   printf("distance %d is minimum \n",i);
                index+=i;
            flag = 1;
            // sleep(1);
            }

        }
        if (flag==1)
        {
       //     printf("ref_points : x :%f  y:%f \n", srv.response.posx[index], srv.response.posy[index]);
            flag=0;
            break;
        }

    }
    // printf("index now:%d \n",index);
}
void Controller::init()
{
    //在这里开始监听进程
    printf("init_finished \n");
    read_thread = std::thread(&Controller::start_Read_thread, this);
    Msg_pub = nh.advertise<geometry_msgs::Twist>("ScoutMotionCmd", 30);
}
void Controller::require_service()
{

    // 实例化srv，设置其request消息的内容，这里request包含两个变量，name和age，见Greeting.srv

    srv.request.a = 1;
    srv.request.stx = 0;
    srv.request.sty = 0;
    srv.request.edx = 5;
    srv.request.edy = 0;

    if (client.call(srv))
    {
        // 注意我们的response部分中的内容只包含一个变量response，另，注意将其转变成字符串
        printf("Finish planning \n");
    }
}
void Controller::require_tf() //放到一个线程里面实现
{
    transmute.lock();
    tf2::Quaternion quat(trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w);

    tf2::Matrix3x3(quat).getRPY(roll, pitch, yam);
    transmute.unlock();
}
void Controller::calc_points()
{
    trans = buffer.lookupTransform("map", "base_footprint", ros::Time(0));
    tf2::Quaternion quat(trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w);
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yam);
    double angle = 0;
    double x_now;
    double y_now;
    double dx;
    double dy;
    double total_distance = 0;
    index0 = 0;
    //calculate theta
    x_now = trans.transform.translation.x;
    y_now = trans.transform.translation.y;

    find_min(x_now, y_now);
    for (int i = 0; (i < point_step) && (index + i + 1 < srv.response.max_num); i++)
    {
        dx = srv.response.posx[index + i + 1] - srv.response.posx[index + i];
        dy = srv.response.posy[index + i + 1] - srv.response.posy[index + i];
        index0 = index + i + 1;
        total_distance += sqrt(dx * dx + dy * dy);

        if (total_distance >= look_ahead_distance)
        {
            break;
        }
        if (index + i + 2 >= srv.response.max_num)
        {
            terminal_flag = 1;
            break;
        }
    }
dx=srv.response.posx[index0]-x_now;
dy=srv.response.posy[index0]-y_now;
    //计算angles，从-pi/2到pi/2
    if (dx == 0 && dy > 0)
        angle = PI / 2;
    else if ((dx) == 0 && (dy) < 0)
        angle = PI / 2;
        else if ((dx) == 0 && (dy) ==0)
        angle=0;
    else
        angle = atan2((dy), (dx));

    //修正角度到四个相限
    if ((dx) < 0 && angle <= 0)
    {
        angle = angle + PI;
    }
    if ((dx) < 0 && angle > 0)
    {
        angle = angle - PI;
    }

    //如果yam的范围是-pi到pi，那么y的范围就是-pi/2到3pi/2

    // 计算角度差，即目标角度与车辆角度之间的角度差，暂时只考虑前进方向，只修正前进方向与目标轨迹之间的角度差，认为只能前进不能后退
    double detaang = 0;
    detaang = angle - yam;

    if (detaang > PI)
    {
        detaang = detaang - 2 * PI;
    }
    if (detaang < -PI)
    {
        detaang = detaang + 2 * PI;
    }
    //printf("the distance is %f \n", sqrt(dx * dx + dy * dy));
    if (sqrt(dx * dx + dy * dy) < 0.01)
    {
        omega = 0;
    }
    else
    {
        omega = (2 * vel_const * sin(detaang)) / sqrt(dx * dx + dy * dy);
        printf("omrga :%f \n",omega);
    }
}
void Controller::calc_track_value()
{
}
double Controller::relu(double x)
{
    double y = 0;
    double sup = 0.1;
    if (x < -sup)
    {
        y = -sup;
    }
    else
    {
        if ((x > -sup) && (x < sup))
        {
            y = x;
        }

        else
        {
            y = sup;
        }
    }
}
void Controller::send_cmd()
{
    cmd_speed.angular.z =omega;
    //   cmd_speed.linear.y = vel_const;
    cmd_speed.linear.y = vel_const;
    if (index + 4 >= srv.response.max_num)
    {
        cmd_speed.angular.z = 0;

        cmd_speed.linear.y = 0;
    }
    Msg_pub.publish(cmd_speed);
    ros::spinOnce();
    omega_last=omega;
}