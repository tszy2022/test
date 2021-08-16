#include <iostream>
#include "Connector.h"
class Connector;
int main()
{
    Connector ioport;
    test_print_all_information(ioport);//打印所有信息
    test_print("ang_vel",ioport);//获取指定信息
    test_print("lin_vel",ioport);//获取指定信息
    test_print("lin_vel",ioport);//获取指定信息
    test_print("battery",ioport);//获取指定信息
    test_cmd("forward",10,ioport);//输入指定命令
    test_cmd("forward",-10,ioport);//输入指定命令
    test_cmd("swing",0.2,ioport);//输入指定命令
}
