#include "Connector.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <mutex>
#include <thread>
#include <aio.h>

Connector::Connector()
{
    m_sockfd=0;
}

Connector::~Connector()
{
    if (m_sockfd!=0) close(m_sockfd);
}
void Connector::print()
{
    printf("hello,world");
}
bool Connector::ConnectToServer(const char *serverip,const int port)
{
    m_sockfd = socket(AF_INET,SOCK_STREAM,0); // 创建客户端的socket

    struct hostent* h; // ip地址信息的数据结构
    if ( (h=gethostbyname(serverip))==0 )
    {
        close(m_sockfd);
        m_sockfd=0;
        return false;
    }

    // 把服务器的地址和端口转换为数据结构
    struct sockaddr_in servaddr;
    memset(&servaddr,0,sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(port);
    memcpy(&servaddr.sin_addr,h->h_addr,h->h_length);

    // 向服务器发起连接请求
    if (connect(m_sockfd,(struct sockaddr *)&servaddr,sizeof(servaddr))!=0)
    {
        close(m_sockfd);
        m_sockfd=0;
        return false;
    }

    return true;
}

int Connector::Send(const void *buf,const int buflen)
{
    return send(m_sockfd,buf,buflen,0);
}

int Connector::Recv(void *buf,const int buflen)
{
    return recv(m_sockfd,buf,buflen,0);
}
int Connector::Read(void *buf,const int buflen)
{
    return read(m_sockfd,buf,buflen);
}
int Connector::Write(void *buf,const int buflen)
{
    return write(m_sockfd,buf,buflen);
}

int Connector::init()
{

    if (ConnectToServer("192.168.1.10",4001)==false)
    {
        printf("TcpClient.ConnectToServer(\"192.168.1.10\",4001) failed,exit...\n");
        return -1;
    }

    else
    {
        unsigned char strbuffer[13]= {0x08,0x00,0x00,0x04,0x21,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
        Send(strbuffer,13);
        unsigned char strbuffer0[13]= {0x08,0x00,0x00,0x01,0x21,0x01,0x01,0x05,0x01,0x05,0x00,0x00,0x00};

        unsigned char strbuffer1[13]= {0x08,0x00,0x00,0x01,0x21,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

        unsigned char strbuffer2[13]= {0x08,0x00,0x00,0x01,0x21,0x01,0x01,0x05,0x01,0x05,0x00,0x00,0x00};

//char strbuf1[8] {\000,\000,\000,\000,\000,\000,\000,\b};
        Send(strbuffer0,13);
        sleep(1);
        Send(strbuffer1,13);
        sleep(1);
        Send(strbuffer2,13);
        sleep(1);
        printf("initialization completed. \n");
        return 1;
//std::thread read_thread();

//std::thread read_thread();



    }
    memset(&aior, 0, sizeof(aior));
    memset(&aiow, 0, sizeof(aiow));
    aior.aio_fildes=m_sockfd;
    aiow.aio_fildes=m_sockfd;
    aior.aio_offset=0;
    aiow.aio_offset=0;
    aior.aio_buf=rec_buffer;
    aiow.aio_buf=snd_buffer;
    aior.aio_nbytes=8192;
    aiow.aio_nbytes=13;
    std::thread(&Connector::start_Read_thread, this);
    //std::thread(&Connector::start_Write_thread, this);

}
void Connector::start_Read_thread()
{
    int ret;
    while(1)
    {
        read_tm.reset();
        scout_state_mutex.lock();
        ret=aio_read(&aior);
        if (ret<0)
        {
            perror("aio_read");
            exit(1);
        }
        while(aio_error(&aior)==EINPROGRESS)
        {
            //   printf("client recv 第 %d 次 \n",++a);
            //     ++a;
        }
        scout_state_mutex.unlock();
        read_tm.sleep_until_ms(10);

//        for(int i{0}; i<=10; ++i)
//        {
//            printf("%0x ",strbuffer[i]);
//        }
        // sleep(1);
//        printf("\n");

    }
}
void Connector::printall()//打印一次所有信息，调试专用函数
{
    const std::lock_guard<std::mutex> lock(scout_state_mutex);
    //读取之前防止翻转状态，上锁
    printf("display current state: \n");
    printf("base_state：%0x \n",scout_state.base_state);
    printf("control_mode：%0x \n",scout_state.control_mode);
    printf("fault_code：%0x \n",scout_state.fault_code);
    printf("battery_voltage：%f volts \n",scout_state.battery_voltage);
    printf("\n \n --motor state----------------------------------------segment-- \n \n");

    printf("FRONT_RIGHT motor_current：%f A \n",scout_state.actuator_states[0].motor_current);
    printf("FRONT_LEFT motor_current：%f A \n",scout_state.actuator_states[1].motor_current);
    printf("REAR_LEFT motor_current：%f A \n",scout_state.actuator_states[2].motor_current);
    printf("REAR_RIGHT motor_current：%f A \n",scout_state.actuator_states[3].motor_current);

    printf("FRONT_RIGHT motor_rpm：%f  \n",scout_state.actuator_states[0].motor_rpm);
    printf("FRONT_LEFT motor_rpm：%f  \n",scout_state.actuator_states[1].motor_rpm);
    printf("REAR_LEFT motor_rpm：%f  \n",scout_state.actuator_states[2].motor_rpm);
    printf("REAR_RIGHT motor_rpm：%f  \n",scout_state.actuator_states[3].motor_rpm);

    printf("FRONT_LEFT motor_rpm：%f  \n",scout_state.actuator_states[0].motor_rpm);
    printf("FRONT_LEFT motor_rpm：%f  \n",scout_state.actuator_states[1].motor_rpm);
    printf("FRONT_LEFT motor_rpm：%f  \n",scout_state.actuator_states[2].motor_rpm);
    printf("FRONT_LEFT motor_rpm：%f  \n",scout_state.actuator_states[3].motor_rpm);


    printf("FRONT_LEFT driver_voltage：%f V \n",scout_state.actuator_states[0].driver_voltage);
    printf("FRONT_LEFT driver_voltage：%f V \n",scout_state.actuator_states[1].driver_voltage);
    printf("FRONT_LEFT driver_voltage：%f V \n",scout_state.actuator_states[2].driver_voltage);
    printf("FRONT_LEFT driver_voltage：%f V \n",scout_state.actuator_states[3].driver_voltage);

    printf("\n \n --light state----------------------------------------segment-- \n \n");

    printf("light_control_enabled %d \n",scout_state.light_control_enabled);
    printf("front_light_state mode:%0x   ***** custom_value:%0x  \n ",scout_state.front_light_state.mode,scout_state.front_light_state.custom_value);
    printf("rear_light_state mode:%0x   ***** custom_value:%0x  \n ",scout_state.rear_light_state.mode,scout_state.rear_light_state.custom_value);

    printf("\n \n --motion state----------------------------------------segment-- \n \n");
    printf("linear_vel:%f m/s \n",scout_state.linear_velocity);
    printf("angular_velocity:%f rad/s \n",scout_state.angular_velocity);

    printf("\n \n --odometer state----------------------------------------segment-- \n \n");
    printf("left_odometry:%f m right_odometry:%f m \n",scout_state.left_odometry,scout_state.right_odometry);

    printf("\n \n --BMS state----------------------------------------segment-- \n \n");
    printf("bms_battery_voltage:%d V \n",scout_state.bms_battery_voltage);
    printf("battery_current:%f A \n",scout_state.battery_current);

    printf("\n \n --temperature----------------------------------------segment-- \n \n");
    printf("FRONT_RIGHT motor_temperature:%f  \n",scout_state.actuator_states[0].motor_temperature);
    printf("FRONT_LEFT motor_temperature:%f  \n",scout_state.actuator_states[1].motor_temperature);
    printf("REAR_LEFT motor_temperature:%f  \n",scout_state.actuator_states[2].motor_temperature);
    printf("REAR_RIGHT motor_temperature:%f  \n",scout_state.actuator_states[3].motor_temperature);

    printf("battery_temperature:%f \n",scout_state.battery_temperature);


//读取之后解锁
    const std::lock_guard<std::mutex> unlock(scout_state_mutex);
    sleep(0.5);
}



void Connector::EncodeCanFrame(const AgxMessage *msg, struct can_frame *tx_frame)
{
    //将msg中的信息传入到要发送的can_frame中
    switch (msg->type)  //判定通信种类，，根据不同的信息种类，将msg中的不同信息传递到can_frame中，并位canid赋值
    {
    // command frame
    case AgxMsgMotionCommand:
    {
        tx_frame->can_id = CAN_MSG_MOTION_COMMAND_ID;
        tx_frame->can_dlc = 8;
        memcpy(tx_frame->data, msg->body.motion_command_msg.raw,
               tx_frame->can_dlc);
        break;
    }
    case AgxMsgLightCommand:
    {
        tx_frame->can_id = CAN_MSG_LIGHT_COMMAND_ID;
        tx_frame->can_dlc = 8;
        memcpy(tx_frame->data, msg->body.light_command_msg.raw,
               tx_frame->can_dlc);
        break;
    }
    case AgxMsgCtrlModeSelect:  //根据can通讯的种类确定canid的种类
    {
        tx_frame->can_id = CAN_MSG_CTRL_MODE_SELECT_ID;
        tx_frame->can_dlc = 8;//帧长度
        memcpy(tx_frame->data, msg->body.ctrl_mode_select_msg.raw,
               tx_frame->can_dlc);//为数组赋值，将要传递的信息录入can框架中
        break;
    }
    case AgxMsgFaultByteReset:
    {
        tx_frame->can_id = CAN_MSG_STATE_RESET_ID;
        tx_frame->can_dlc = 8;
        memcpy(tx_frame->data, msg->body.state_reset_msg.raw, tx_frame->can_dlc);
        break;
    }
    // state feedback frame
    case AgxMsgSystemState:
    {
        tx_frame->can_id = CAN_MSG_SYSTEM_STATE_ID;
        tx_frame->can_dlc = 8;
        memcpy(tx_frame->data, msg->body.system_state_msg.raw, tx_frame->can_dlc);
        break;
    }
    case AgxMsgMotionState:
    {
        tx_frame->can_id = CAN_MSG_MOTION_STATE_ID;
        tx_frame->can_dlc = 8;
        memcpy(tx_frame->data, msg->body.motion_state_msg.raw, tx_frame->can_dlc);
        break;
    }
    case AgxMsgLightState:
    {
        tx_frame->can_id = CAN_MSG_LIGHT_STATE_ID;
        tx_frame->can_dlc = 8;
        memcpy(tx_frame->data, msg->body.light_state_msg.raw, tx_frame->can_dlc);
        break;
    }
    case AgxMsgOdometry:
    {
        tx_frame->can_id = CAN_MSG_ODOMETRY_ID;
        tx_frame->can_dlc = 8;
        memcpy(tx_frame->data, msg->body.odometry_msg.raw, tx_frame->can_dlc);
        break;
    }
    case AgxMsgActuatorHSState:
    {
        switch (msg->body.actuator_hs_state_msg.motor_id)
        {
        case ACTUATOR1_ID:
        {
            tx_frame->can_id = CAN_MSG_ACTUATOR1_HS_STATE_ID;
            break;
        }
        case ACTUATOR2_ID:
        {
            tx_frame->can_id = CAN_MSG_ACTUATOR2_HS_STATE_ID;
            break;
        }
        case ACTUATOR3_ID:
        {
            tx_frame->can_id = CAN_MSG_ACTUATOR3_HS_STATE_ID;
            break;
        }
        case ACTUATOR4_ID:
        {
            tx_frame->can_id = CAN_MSG_ACTUATOR4_HS_STATE_ID;
            break;
        }
        }
        tx_frame->can_dlc = 8;
        memcpy(tx_frame->data, msg->body.actuator_hs_state_msg.data.raw,
               tx_frame->can_dlc);
        break;
    }
    case AgxMsgActuatorLSState:
    {
        switch (msg->body.actuator_ls_state_msg.motor_id)
        {
        case ACTUATOR1_ID:
        {
            tx_frame->can_id = CAN_MSG_ACTUATOR1_LS_STATE_ID;
            break;
        }
        case ACTUATOR2_ID:
        {
            tx_frame->can_id = CAN_MSG_ACTUATOR2_LS_STATE_ID;
            break;
        }
        case ACTUATOR3_ID:
        {
            tx_frame->can_id = CAN_MSG_ACTUATOR3_LS_STATE_ID;
            break;
        }
        case ACTUATOR4_ID:
        {
            tx_frame->can_id = CAN_MSG_ACTUATOR4_LS_STATE_ID;
            break;
        }
        }
        tx_frame->can_dlc = 8;
        memcpy(tx_frame->data, msg->body.actuator_ls_state_msg.data.raw,
               tx_frame->can_dlc);
        break;
    }
    case AgxMsgParkModeSelect:
    {
        tx_frame->can_id = CAN_MSG_PARK_COMMAND_ID;
        tx_frame->can_dlc = 8;
        memcpy(tx_frame->data, msg->body.park_control_msg.raw, tx_frame->can_dlc);
        break;
    }
    default:
        break;
    }
    //   tx_frame->data[7] =
    //       CalcCanFrameChecksum(tx_frame->can_id, tx_frame->data,
    //       tx_frame->can_dlc);
}
void Connector::copy_to_can_frame(can_frame *rx_frame, uint8_t *msg)
{
    ++msg;
    rx_frame->can_id=(
                         static_cast<uint32_t>(msg[0]) |
                         static_cast<uint32_t>(msg[2]) << 8 |
                         static_cast<uint32_t>(msg[3]));
//          printf("canID1: %0x finish \n",rx_frame->can_id);
    for(int count{0}; count<=7; ++count)
    {
        rx_frame->data[count]=msg[count+4];
    }

}
void Connector::convert_data_once(const AgxMessage &status_msg,ScoutState &state)//将status转换为scout类型到数据
{
    switch (status_msg.type)  //根据msg的种类确定进行什么操作
    {
    case AgxMsgSystemState:  //如果回馈的是系统信息
    {
        // std::cout << "system status feedback received" << std::endl;
        const SystemStateMessage &msg = status_msg.body.system_state_msg;//定义一个msg结构继承传来的信息内容
        state.control_mode = msg.state.control_mode;//用得到的信息更新系统状态
        state.base_state = msg.state.vehicle_state;
        state.battery_voltage =
            (static_cast<uint16_t>(msg.state.battery_voltage.low_byte) |
             static_cast<uint16_t>(msg.state.battery_voltage.high_byte) << 8) /
            10.0;//把八位的两字节电池典雅信息转变为电池的实际电压
        state.fault_code = msg.state.fault_code;
        break;
    }
    case AgxMsgMotionState:  //如果回馈的是运动信息
    {
        // std::cout << "motion control feedback received" << std::endl;
        const MotionStateMessage &msg = status_msg.body.motion_state_msg;
        state.linear_velocity =//更新速度信息
            static_cast<int16_t>(
                static_cast<uint16_t>(msg.state.linear_velocity.low_byte) |
                static_cast<uint16_t>(msg.state.linear_velocity.high_byte) << 8) /
            1000.0;
        state.angular_velocity =//更新角速度信息
            static_cast<int16_t>(
                static_cast<uint16_t>(msg.state.angular_velocity.low_byte) |
                static_cast<uint16_t>(msg.state.angular_velocity.high_byte)
                << 8) /
            1000.0;
        break;
    }
    case AgxMsgLightState:  //如果回馈的是灯的控制信息
    {
        // std::cout << "light control feedback received" << std::endl;
        const LightStateMessage &msg = status_msg.body.light_state_msg;
        if (msg.state.light_ctrl_enabled == LIGHT_CTRL_DISABLE)
            state.light_control_enabled = false;
        else
            state.light_control_enabled = true;//更新状态信息
        state.front_light_state.mode = msg.state.front_light_mode;
        state.front_light_state.custom_value = msg.state.front_light_custom;
        state.rear_light_state.mode = msg.state.rear_light_mode;
        state.rear_light_state.custom_value = msg.state.rear_light_custom;
        break;
    }
    case AgxMsgActuatorHSState:  //如果回馈的是电机的信息
    {
        // std::cout << "actuator hs feedback received" << std::endl;
        const ActuatorHSStateMessage &msg = status_msg.body.actuator_hs_state_msg;
        state.actuator_states[msg.motor_id].motor_current =//更新电机电流
            (static_cast<uint16_t>(msg.data.state.current.low_byte) |
             static_cast<uint16_t>(msg.data.state.current.high_byte) << 8) /
            10.0;
        state.actuator_states[msg.motor_id].motor_rpm = static_cast<int16_t>(//更新电机转速
                    static_cast<uint16_t>(msg.data.state.rpm.low_byte) |
                    static_cast<uint16_t>(msg.data.state.rpm.high_byte) << 8);
        state.actuator_states[msg.motor_id].motor_pulses = static_cast<int32_t>(
                    static_cast<uint32_t>(msg.data.state.pulse_count.lsb) |
                    static_cast<uint32_t>(msg.data.state.pulse_count.low_byte) << 8 |
                    static_cast<uint32_t>(msg.data.state.pulse_count.high_byte) << 16 |
                    static_cast<uint32_t>(msg.data.state.pulse_count.msb) << 24);
        break;
    }
    case AgxMsgActuatorLSState:  //如果回馈的是电机信息
    {
        // std::cout << "actuator ls feedback received" << std::endl;
        const ActuatorLSStateMessage &msg = status_msg.body.actuator_ls_state_msg;
        for (int i = 0; i < 2; ++i)
        {
            state.actuator_states[msg.motor_id].driver_voltage =//更新电机驱动电压
                (static_cast<uint16_t>(msg.data.state.driver_voltage.low_byte) |
                 static_cast<uint16_t>(msg.data.state.driver_voltage.high_byte)
                 << 8) /
                10.0;
            state.actuator_states[msg.motor_id]
            .driver_temperature = static_cast<int16_t>(//更新电机温度
                                      static_cast<uint16_t>(msg.data.state.driver_temperature.low_byte) |
                                      static_cast<uint16_t>(msg.data.state.driver_temperature.high_byte)
                                      << 8);
            state.actuator_states[msg.motor_id].motor_temperature =
                msg.data.state.motor_temperature;
            state.actuator_states[msg.motor_id].driver_state =
                msg.data.state.driver_state;
        }
        break;
    }
    case AgxMsgOdometry:  //如果获取的是里程计信息
    {
        // std::cout << "Odometer msg feedback received" << std::endl;
        const OdometryMessage &msg = status_msg.body.odometry_msg;
        state.right_odometry = static_cast<int32_t>(
                                   (static_cast<uint32_t>(msg.state.right_wheel.lsb)) |
                                   (static_cast<uint32_t>(msg.state.right_wheel.low_byte) << 8) |
                                   (static_cast<uint32_t>(msg.state.right_wheel.high_byte) << 16) |
                                   (static_cast<uint32_t>(msg.state.right_wheel.msb) << 24));
        state.left_odometry = static_cast<int32_t>(
                                  (static_cast<uint32_t>(msg.state.left_wheel.lsb)) |
                                  (static_cast<uint32_t>(msg.state.left_wheel.low_byte) << 8) |
                                  (static_cast<uint32_t>(msg.state.left_wheel.high_byte) << 16) |
                                  (static_cast<uint32_t>(msg.state.left_wheel.msb) << 24));
        break;
    }
    case AgxMsgBmsDate:  //如果获取的电池信息
    {
        // std::cout << "Odometer msg feedback received" << std::endl;
        const BMSDateMessage &msg = status_msg.body.bms_date_msg;
        state.SOC = msg.state.battery_SOC;
        state.SOH = msg.state.battery_SOH;
        state.bms_battery_voltage = static_cast<int16_t>(
                                        (static_cast<uint16_t>(msg.state.battery_voltage.low_byte)) |
                                        (static_cast<uint16_t>(msg.state.battery_voltage.high_byte) << 8));
        state.battery_current = static_cast<int16_t>(
                                    (static_cast<uint16_t>(msg.state.battery_current.low_byte)) |
                                    (static_cast<uint16_t>(msg.state.battery_current.high_byte) << 8));
        state.battery_temperature = static_cast<int16_t>(
                                        (static_cast<uint16_t>(msg.state.battery_temperature.low_byte)) |
                                        (static_cast<uint16_t>(msg.state.battery_temperature.high_byte) << 8));
        break;
    }
    case AgxMsgBmsStatus:  //如果获取的使滇池的报警信息
    {
        // std::cout << "Odometer msg feedback received" << std::endl;
        const BMSStatusMessage &msg = status_msg.body.bms_status_msg;
        state.Alarm_Status_1 = msg.state.Alarm_Status_1;
        state.Alarm_Status_2 = msg.state.Alarm_Status_2;
        state.Warning_Status_1 = msg.state.Warning_Status_1;
        state.Warning_Status_2 = msg.state.Warning_Status_2;
    }

    }
}


void Connector::unpack_all() //打印一次所有信息，调试专用函数
{
    for (int count{ 0 }; count <= 100; ++count)
    {
        // printf("%d",count);
        if (Read(rec_buffer0,sizeof(rec_buffer0))<=0) break;
        //    memcpy(&mgs.raw, rec_buffer0+5,8 * sizeof(uint8_t));
        //   printf("%0x  \n",&(rec_buffer0[0]));
        copy_to_can_frame(can_frame_pt, &(rec_buffer0[0]));
        DecodeCanFrame(can_frame_pt, agx_msg_pt);
        convert_data_once(agx_msg,scout_state);
        //上面三步之后，所有状态储存在scout_state里面

//            for (int j{ 0 }; j <=12; ++j)
//                {
//                    printf("%0x  ",rec_buffer0[j]);
//                }
//                printf("****\n");
//                   for (int j{ 0 }; j <=7; ++j)
//                {
//                   printf("%0x ",mgs.raw[j]);
//                   //printf("%0x  ",mgs.cmd);
//
//                }
//                printf("****\n");
//        //       printf("%0x ",can_frame_pt->can_id);
//
//                printf("end  \n");
//                sleep(1);
    }
    printall();

}
void Connector::SetLightCommand(const ScoutLightCmd &cmd)
{
    static uint8_t light_cmd_count = 0;
    SendLightCmd(cmd, light_cmd_count++);
}
void Connector::SendLightCmd(const ScoutLightCmd &lcmd, uint8_t count)
{
    //将灯的控制命令传递给msg，然后填充入can框架中，最后以流的形式发送
    AgxMessage l_msg;//定义一个msg类对象
    l_msg.type = AgxMsgLightCommand;//定义通信功能种类
    memset(l_msg.body.light_command_msg.raw, 0, 8);//初始化数组

    if (lcmd.enable_ctrl)  //如果can控制使能
    {
        l_msg.body.light_command_msg.cmd.light_ctrl_enabled = LIGHT_CTRL_ENABLE;

        l_msg.body.light_command_msg.cmd.front_light_mode =//将cmd中的控制指令传递到msg中
            static_cast<uint8_t>(lcmd.front_mode);
        l_msg.body.light_command_msg.cmd.front_light_custom =
            lcmd.front_custom_value;
        l_msg.body.light_command_msg.cmd.rear_light_mode =
            static_cast<uint8_t>(lcmd.rear_mode);
        l_msg.body.light_command_msg.cmd.rear_light_custom = lcmd.rear_custom_value;
    }
    else
    {
        l_msg.body.light_command_msg.cmd.light_ctrl_enabled = LIGHT_CTRL_DISABLE;
    }

    l_msg.body.light_command_msg.cmd.count = count;

    // send to can bus
    can_frame l_frame;//把msg填充到参框架中
    EncodeCanFrame(&l_msg, &l_frame);
    copy_to_buffer(&l_frame);//将can信息以流的形式发送
}
void Connector::copy_to_buffer(can_frame *rx_frame)
{
    uint8_t buf[13] {0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    for(int i=0; i<=7; ++i)
    {
        buf[i+5]=rx_frame->data[i];
    }
    uint32_t pt0;
    pt0=rx_frame->can_id;
    char* pt{(char*)&pt0};
    buf[1]=pt[3];
    buf[2]=pt[2];
    buf[3]=pt[1];
    buf[4]=pt[0];
    Send(buf,13);
    sleep(0.1);


}
void Connector::cmd_test()
{
//灯光控制指令,所有指令按照以下标准赋值，不要按照ugvsdk里面到代码赋值
    int i {0};
    ScoutLightCmd cmd {};
    cmd.enable_ctrl=1;
    cmd.front_mode=ScoutLightCmd::LightMode::CONST_OFF;
    cmd.rear_mode=ScoutLightCmd::LightMode::CUSTOM;
    cmd.front_custom_value=0x01;
    cmd.rear_custom_value=0x64;
    SetLightCommand(cmd);
    printf("finish sending Lightcmd \n");
    while(1)
    {
        cmd.front_mode=ScoutLightCmd::LightMode::CONST_ON;
        cmd.rear_mode=ScoutLightCmd::LightMode::CONST_ON;
        SetLightCommand(cmd);
        sleep(1);
        cmd.front_mode=ScoutLightCmd::LightMode::CONST_OFF;
        cmd.rear_mode=ScoutLightCmd::LightMode::CONST_OFF;
        SetLightCommand(cmd);
        printf("finish sending Lightcmd \n");
        sleep(1);

    }

//定义运动控制参数
//定义控制指令类对象
    current_motion_cmd_.angular_velocity=0;
    current_motion_cmd_.linear_velocity=1;
    sleep(0.1);
    while(1)
    {


        SendMotionCmd();
        printf("sending motioncmd %d \n",i);
//i++;
    }

    printf("finish sending motioncmd \n");

}
void Connector::SendMotionCmd()
{
    //定义运动控制指令发送函数
    // motion control message
    AgxMessage m_msg;//定义msg
    m_msg.type = AgxMsgMotionCommand;//选择can'通信种类为运动控制
// memset(m_msg.body.motion_command_msg.raw, 0, 8);//初始化参信息

    //motion_cmd_mutex_.lock();//上锁保证在进行如下操作的时候相关的变量值不会被其他进程修改
    int16_t linear_cmd =
        static_cast<int16_t>(current_motion_cmd_.linear_velocity * 1000);//将线速度和角速度转变为can协议定义的数值
    int16_t angular_cmd =
        static_cast<int16_t>(current_motion_cmd_.angular_velocity * 1000);
    int16_t lateral_cmd =
        static_cast<int16_t>(current_motion_cmd_.lateral_velocity * 1000);
    //motion_cmd_mutex_.unlock();//解索

    // SendControlCmd();
    m_msg.body.motion_command_msg.cmd.linear_velocity.high_byte =//把上面得到的十六位速度信息转变为量子节的把为速度信息
        (static_cast<uint16_t>(linear_cmd) >> 8) & 0x00ff;
    m_msg.body.motion_command_msg.cmd.linear_velocity.low_byte =
        (static_cast<uint16_t>(linear_cmd) >> 0) & 0x00ff;
    m_msg.body.motion_command_msg.cmd.angular_velocity.high_byte =
        (static_cast<uint16_t>(angular_cmd) >> 8) & 0x00ff;
    m_msg.body.motion_command_msg.cmd.angular_velocity.low_byte =
        (static_cast<uint16_t>(angular_cmd) >> 0) & 0x00ff;
    m_msg.body.motion_command_msg.cmd.lateral_velocity.high_byte =
        (static_cast<uint16_t>(lateral_cmd) >> 8) & 0x00ff;
    m_msg.body.motion_command_msg.cmd.lateral_velocity.low_byte =
        (static_cast<uint16_t>(lateral_cmd) >> 0) & 0x00ff;

    // send to can bus
    can_frame m_frame;//定义can框架对象
    EncodeCanFrame(&m_msg, &m_frame);//打包can信息
    copy_to_buffer(&m_frame);
}




bool Connector::DecodeCanFrame(const struct can_frame *rx_frame, AgxMessage *msg)
{
    msg->type = AgxMsgUnkonwn;

    switch (rx_frame->can_id)
    {
    // command frame
    case CAN_MSG_MOTION_COMMAND_ID:
    {
        msg->type = AgxMsgMotionCommand;
        memcpy(msg->body.motion_command_msg.raw, rx_frame->data,
               rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_LIGHT_COMMAND_ID:
    {
        msg->type = AgxMsgLightCommand;
        memcpy(msg->body.light_command_msg.raw, rx_frame->data,
               rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_RC_STATE_ID:
    {
        msg->type = AgxMsgRcState;
        memcpy(msg->body.rc_state_msg.raw, rx_frame->data,
               rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_CTRL_MODE_SELECT_ID:
    {
        msg->type = AgxMsgCtrlModeSelect;
        memcpy(msg->body.ctrl_mode_select_msg.raw, rx_frame->data,
               rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_STATE_RESET_ID:
    {
        msg->type = AgxMsgFaultByteReset;
        memcpy(msg->body.state_reset_msg.raw, rx_frame->data,
               rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    // state feedback frame
    case CAN_MSG_SYSTEM_STATE_ID:
    {
        msg->type = AgxMsgSystemState;
        memcpy(msg->body.system_state_msg.raw, rx_frame->data,
               rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_MOTION_STATE_ID:
    {
        msg->type = AgxMsgMotionState;
        memcpy(msg->body.motion_state_msg.raw, rx_frame->data,
               rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_LIGHT_STATE_ID:
    {
        msg->type = AgxMsgLightState;
        memcpy(msg->body.light_state_msg.raw, rx_frame->data,
               rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_ODOMETRY_ID:
    {
        msg->type = AgxMsgOdometry;
        memcpy(msg->body.odometry_msg.raw, rx_frame->data,
               rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_ACTUATOR1_HS_STATE_ID:
    case CAN_MSG_ACTUATOR2_HS_STATE_ID:
    case CAN_MSG_ACTUATOR3_HS_STATE_ID:
    case CAN_MSG_ACTUATOR4_HS_STATE_ID:
    {
        msg->type = AgxMsgActuatorHSState;
        msg->body.actuator_hs_state_msg.motor_id =
            (uint8_t)(rx_frame->can_id - CAN_MSG_ACTUATOR1_HS_STATE_ID);
        memcpy(msg->body.actuator_hs_state_msg.data.raw, rx_frame->data,
               rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_ACTUATOR1_LS_STATE_ID:
    case CAN_MSG_ACTUATOR2_LS_STATE_ID:
    case CAN_MSG_ACTUATOR3_LS_STATE_ID:
    case CAN_MSG_ACTUATOR4_LS_STATE_ID:
    {
        msg->type = AgxMsgActuatorLSState;
        msg->body.actuator_ls_state_msg.motor_id =
            (uint8_t)(rx_frame->can_id - CAN_MSG_ACTUATOR1_LS_STATE_ID);
        memcpy(msg->body.actuator_ls_state_msg.data.raw, rx_frame->data,
               rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_BMS_DATE_ID:
    {
        msg->type = AgxMsgBmsDate;
        memcpy(msg->body.bms_date_msg.raw, rx_frame->data,
               rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_BMS_STATUES_ID:
    {
        msg->type = AgxMsgBmsStatus;
        memcpy(msg->body.bms_status_msg.raw, rx_frame->data,
               rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    default:
        break;
    }

    return true;
}
