#include <iostream>
#include "Connector.h"
//class Connector;
int main()
{
    //uint8_t a[2] {0,1};
    Connector ioport;
    //ioport.print();
    ioport.init();
    //ioport.unpack_all();
    //ioport.cmd_test();
    //can_frame *pt=&ioport.canframe;
    //ioport.copy_to_can_frame(pt,&a[0]);
   //ioport.unpack_all();
   test_print_all_information(ioport);//打印所有信息
   //打印指定信息自行在scout——state中获取
   // test_print("ang_vel",ioport);//获取指定信息
   // test_print("lin_vel",ioport);//获取指定信息
   // test_print("lin_vel",ioport);//获取指定信息
   // test_print("battery",ioport);//获取指定信息

   // test_cmd("forward",10,ioport);//输入指定命令
   // test_cmd("forward",-10,ioport);//输入指定命令
    //test_cmd("swing",0.2,ioport);//输入指定命令
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
