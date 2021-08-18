#include "Connector.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>


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
  { close(m_sockfd); m_sockfd=0; return false; }

  // 把服务器的地址和端口转换为数据结构
  struct sockaddr_in servaddr;
  memset(&servaddr,0,sizeof(servaddr));
  servaddr.sin_family = AF_INET;
  servaddr.sin_port = htons(port);
  memcpy(&servaddr.sin_addr,h->h_addr,h->h_length);

  // 向服务器发起连接请求
  if (connect(m_sockfd,(struct sockaddr *)&servaddr,sizeof(servaddr))!=0)
  { close(m_sockfd); m_sockfd=0; return false; }

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
int Connector::init()
{

   if (ConnectToServer("192.168.1.10",4001)==false)
  { printf("TcpClient.ConnectToServer(\"192.168.1.10\",4001) failed,exit...\n"); return -1; }

    else
    {unsigned char strbuffer[13]={0x08,0x00,0x00,0x04,0x21,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    Send(strbuffer,13);
unsigned char strbuffer0[13]={0x08,0x00,0x00,0x01,0x21,0x01,0x01,0x05,0x01,0x05,0x00,0x00,0x00};

unsigned char strbuffer1[13]={0x08,0x00,0x00,0x01,0x21,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

unsigned char strbuffer2[13]={0x08,0x00,0x00,0x01,0x21,0x01,0x01,0x05,0x01,0x05,0x00,0x00,0x00};

//char strbuf1[8] {\000,\000,\000,\000,\000,\000,\000,\b};
   Send(strbuffer0,13);
   sleep(1);
    Send(strbuffer1,13);
   sleep(1);
    Send(strbuffer2,13);
   printf("initialization completed. \n");
   return 1;
    }
}

void Connector::printall()//打印一次所有信息，调试专用函数
{
	printf("display current state: \n");
	printf("base_state：%0x \n",scout_state.base_state);
	printf("control_mode：%0x \n",scout_state.control_mode);
	printf("fault_code：%0x \n",scout_state.fault_code);
	printf("battery_voltage：%f volts \n",scout_state.battery_voltage);
	printf("/n /n --motor state----------------------------------------segment-- /n /n");
	
	printf("FRONT_RIGHT motor_current：%f A \n",scout_state.actuator_states[FRONT_RIGHT].motor_current);
	printf("FRONT_LEFT motor_current：%f A \n",scout_state.actuator_states[FRONT_LEFT].motor_current);
	printf("REAR_LEFT motor_current：%f A \n",scout_state.actuator_states[REAR_LEFT].motor_current);
	printf("REAR_RIGHT motor_current：%f A \n",scout_state.actuator_states[REAR_RIGHT].motor_current);

	printf("FRONT_RIGHT motor_rpm：%f  \n",scout_state.actuator_states[FRONT_RIGHT].motor_rpm);
	printf("FRONT_LEFT motor_rpm：%f  \n",scout_state.actuator_states[FRONT_LEFT].motor_rpm);
	printf("REAR_LEFT motor_rpm：%f  \n",scout_state.actuator_states[REAR_LEFT].motor_rpm);
	printf("REAR_RIGHT motor_rpm：%f  \n",scout_state.actuator_states[REAR_RIGHT].motor_rpm);
	
	printf("FRONT_LEFT motor_rpm：%f  \n",scout_state.actuator_states[FRONT_LEFT].motor_rpm);
	printf("FRONT_LEFT motor_rpm：%f  \n",scout_state.actuator_states[FRONT_LEFT].motor_rpm);
	printf("FRONT_LEFT motor_rpm：%f  \n",scout_state.actuator_states[FRONT_LEFT].motor_rpm);
	printf("FRONT_LEFT motor_rpm：%f  \n",scout_state.actuator_states[FRONT_LEFT].motor_rpm);
	
	
	printf("FRONT_LEFT driver_voltage：%f V \n",scout_state.actuator_states[FRONT_LEFT].driver_voltage);
	printf("FRONT_LEFT driver_voltage：%f V \n",scout_state.actuator_states[FRONT_LEFT].driver_voltage);
	printf("FRONT_LEFT driver_voltage：%f V \n",scout_state.actuator_states[FRONT_LEFT].driver_voltage);
	printf("FRONT_LEFT driver_voltage：%f V \n",scout_state.actuator_states[FRONT_LEFT].driver_voltage);
	
	printf("/n /n --light state----------------------------------------segment-- /n /n");

	printf("light_control_enabled %d \n",scout_state.light_control_enabled);	
	printf("front_light_state mode:%0x   ***** custom_value:%0x  \n ",scout_state.front_light_state.mode,scout_state.front_light_state.custom_value);	
	printf("rear_light_state mode:%0x   ***** custom_value:%0x  \n ",scout_state.rear_light_state.mode,scout_state.rear_light_state.custom_value);		
		
	printf("/n /n --motion state----------------------------------------segment-- /n /n");
	printf("linear_vel:%f m/s \n",scout_state.linear_velocity);
	printf("angular_velocity:%f rad/s \n",scout_state.angular_velocity);
	
	printf("/n /n --odometer state----------------------------------------segment-- /n /n");
	printf("left_odometry:%f m right_odometry:%f m \n",scout_state.left_odometry,scout_state.right_odometry);

	printf("/n /n --BMS state----------------------------------------segment-- /n /n");
	printf("bms_battery_voltage:%f V \n",scout_state.bms_battery_voltage);
	printf("battery_current:%f A \n");

	printf("/n /n --temperature----------------------------------------segment-- /n /n");
	printf("FRONT_RIGHT motor_temperature:%f  \n",scout_state.actuator_states[FRONT_RIGHT].motor_temperature);
	printf("FRONT_LEFT motor_temperature:%f  \n",scout_state.actuator_states[FRONT_LEFT].motor_temperature);
	printf("REAR_LEFT motor_temperature:%f  \n",scout_state.actuator_states[REAR_LEFT].motor_temperature);
	printf("REAR_RIGHT motor_temperature:%f  \n",scout_state.actuator_states[REAR_RIGHT].motor_temperature);

	printf("battery_temperature:%f \n",scout_state.battery_temperature);
	
	

}
void Connector::copy_to_can_frame(can_frame *rx_frame, uint8_t *msg)
{
            ++msg;
           rx_frame->can_id=(
          static_cast<uint32_t>(msg[0]) |
          static_cast<uint32_t>(msg[2]) << 8 |
          static_cast<uint32_t>(msg[3]));
//          printf("canID1: %0x finish \n",rx_frame->can_id);
        for(int count{0};count<=7;++count)
            {
                rx_frame->data[count]=msg[count+4];
            }
//                    for(int count{0};count<=7;++count)
//            {
//              printf("%0x ",rx_frame->data[count]);
//            }

//switch (rx_frame->can_id)
//{
//case CAN_MSG_MOTION_COMMAND_ID:{ printf("getID!CAN_MSG_MOTION_COMMAND_ID\n");break;};
//case CAN_MSG_LIGHT_COMMAND_ID:{ printf("getID!CAN_MSG_LIGHT_COMMAND_ID\n");break;};
//case CAN_MSG_PARK_COMMAND_ID:{ printf("getID!CAN_MSG_PARK_COMMAND_ID\n");break;};
//case CAN_MSG_SYSTEM_STATE_ID:{ printf("getID!CAN_MSG_SYSTEM_STATE_ID\n");break;};
//case CAN_MSG_MOTION_STATE_ID:{ printf("getID!CAN_MSG_MOTION_STATE_ID\n");break;};
//case CAN_MSG_LIGHT_STATE_ID:{ printf("getID!CAN_MSG_LIGHT_STATE_ID\n");break;};
//case CAN_MSG_RC_STATE_ID:{ printf("getID!CAN_MSG_RC_STATE_ID\n");break;};
//case CAN_MSG_ACTUATOR1_HS_STATE_ID:{ printf("getID!CAN_MSG_ACTUATOR1_HS_STATE_ID\n");break;};
//case CAN_MSG_ACTUATOR2_HS_STATE_ID:{ printf("getID!CAN_MSG_ACTUATOR2_HS_STATE_ID\n");break;};
//case CAN_MSG_ACTUATOR3_HS_STATE_ID:{ printf("getID!CAN_MSG_ACTUATOR3_HS_STATE_ID\n");break;};
//case CAN_MSG_ACTUATOR4_HS_STATE_ID:{ printf("getID!CAN_MSG_ACTUATOR4_HS_STATE_ID\n");break;};
//case CAN_MSG_ACTUATOR1_LS_STATE_ID:{ printf("getID!CAN_MSG_ACTUATOR1_LS_STATE_ID\n");break;};
//case CAN_MSG_ACTUATOR2_LS_STATE_ID:{ printf("getID!CAN_MSG_ACTUATOR2_LS_STATE_ID\n");break;};
//case CAN_MSG_ACTUATOR3_LS_STATE_ID:{ printf("getID!CAN_MSG_ACTUATOR3_LS_STATE_ID\n");break;};
//case CAN_MSG_ACTUATOR4_LS_STATE_ID:{ printf("getID!CAN_MSG_ACTUATOR4_LS_STATE_ID\n");break;};
//case CAN_MSG_ODOMETRY_ID:{ printf("getID!CAN_MSG_ODOMETRY_ID\n");break;};
//case CAN_MSG_BMS_DATE_ID:{ printf("getID!CAN_MSG_BMS_DATE_ID\n");break;};
//case CAN_MSG_BMS_STATUES_ID:{ printf("getID!CAN_MSG_BMS_STATUES_ID\n");break;};
//case CAN_MSG_VERSION_QUERY_ID:{ printf("getID!CAN_MSG_VERSION_QUERY_ID\n");break;};
//case CAN_MSG_PLATFORM_VERSION_ID:{ printf("getID!CAN_MSG_PLATFORM_VERSION_ID\n");break;};
//case CAN_MSG_CTRL_MODE_SELECT_ID:{ printf("getID!CAN_MSG_CTRL_MODE_SELECT_ID\n");break;};
//case CAN_MSG_STEER_NEUTRAL_RESET_ID:{ printf("getID!CAN_MSG_STEER_NEUTRAL_RESET_ID\n");break;};
//case CAN_MSG_STEER_NEUTRAL_RESET_ACK_ID:{ printf("getID!CAN_MSG_STEER_NEUTRAL_RESET_ACK_ID\n");break;};
//case CAN_MSG_STATE_RESET_ID:{ printf("getID!CAN_MSG_STATE_RESET_ID\n");break;};
//
//}

}


void Connector::convert_data_once(const AgxMessage &status_msg,ScoutState &state)
                                 ScoutState &state)

{
switch (status_msg.type) 
 {
    case AgxMsgSystemState: {
      // std::cout << "system status feedback received" << std::endl;
      const SystemStateMessage &msg = status_msg.body.system_state_msg;
      state.control_mode = msg.state.control_mode;
      state.base_state = msg.state.vehicle_state;
      state.battery_voltage =
          (static_cast<uint16_t>(msg.state.battery_voltage.low_byte) |
           static_cast<uint16_t>(msg.state.battery_voltage.high_byte) << 8) /
          10.0;
      state.fault_code = msg.state.fault_code;
      break;
    }
    case AgxMsgMotionState: {
      // std::cout << "motion control feedback received" << std::endl;
      const MotionStateMessage &msg = status_msg.body.motion_state_msg;
      state.linear_velocity =
          static_cast<int16_t>(
              static_cast<uint16_t>(msg.state.linear_velocity.low_byte) |
              static_cast<uint16_t>(msg.state.linear_velocity.high_byte) << 8) /
          1000.0;
      state.angular_velocity =
          static_cast<int16_t>(
              static_cast<uint16_t>(msg.state.angular_velocity.low_byte) |
              static_cast<uint16_t>(msg.state.angular_velocity.high_byte)
                  << 8) /
          1000.0;
      break;
    }
    case AgxMsgLightState: {
      // std::cout << "light control feedback received" << std::endl;
      const LightStateMessage &msg = status_msg.body.light_state_msg;
      if (msg.state.light_ctrl_enabled == LIGHT_CTRL_DISABLE)
        state.light_control_enabled = false;
      else
        state.light_control_enabled = true;
      state.front_light_state.mode = msg.state.front_light_mode;
      state.front_light_state.custom_value = msg.state.front_light_custom;
      state.rear_light_state.mode = msg.state.rear_light_mode;
      state.rear_light_state.custom_value = msg.state.rear_light_custom;
      break;
    }
    case AgxMsgActuatorHSState: {
      // std::cout << "actuator hs feedback received" << std::endl;
      const ActuatorHSStateMessage &msg = status_msg.body.actuator_hs_state_msg;
      state.actuator_states[msg.motor_id].motor_current =
          (static_cast<uint16_t>(msg.data.state.current.low_byte) |
           static_cast<uint16_t>(msg.data.state.current.high_byte) << 8) /
          10.0;
      state.actuator_states[msg.motor_id].motor_rpm = static_cast<int16_t>(
          static_cast<uint16_t>(msg.data.state.rpm.low_byte) |
          static_cast<uint16_t>(msg.data.state.rpm.high_byte) << 8);
      state.actuator_states[msg.motor_id].motor_pulses = static_cast<int32_t>(
          static_cast<uint32_t>(msg.data.state.pulse_count.lsb) |
          static_cast<uint32_t>(msg.data.state.pulse_count.low_byte) << 8 |
          static_cast<uint32_t>(msg.data.state.pulse_count.high_byte) << 16 |
          static_cast<uint32_t>(msg.data.state.pulse_count.msb) << 24);
      break;
    }
    case AgxMsgActuatorLSState: {
      // std::cout << "actuator ls feedback received" << std::endl;
      const ActuatorLSStateMessage &msg = status_msg.body.actuator_ls_state_msg;
      for (int i = 0; i < 2; ++i) {
        state.actuator_states[msg.motor_id].driver_voltage =
            (static_cast<uint16_t>(msg.data.state.driver_voltage.low_byte) |
             static_cast<uint16_t>(msg.data.state.driver_voltage.high_byte)
                 << 8) /
            10.0;
        state.actuator_states[msg.motor_id]
            .driver_temperature = static_cast<int16_t>(
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
    case AgxMsgOdometry: {
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
    case AgxMsgBmsDate: {
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
    case AgxMsgBmsStatus: {
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
//到此为止100个循环后读取数据并且打印
}

bool Connector::DecodeCanFrame(const struct can_frame *rx_frame, AgxMessage *msg) 
{
  msg->type = AgxMsgUnkonwn;

  switch (rx_frame->can_id) {
    // command frame
    case CAN_MSG_MOTION_COMMAND_ID: {
      msg->type = AgxMsgMotionCommand;
      memcpy(msg->body.motion_command_msg.raw, rx_frame->data,
             rx_frame->can_dlc * sizeof(uint8_t));
      break;
    }
    case CAN_MSG_LIGHT_COMMAND_ID: {
      msg->type = AgxMsgLightCommand;
      memcpy(msg->body.light_command_msg.raw, rx_frame->data,
             rx_frame->can_dlc * sizeof(uint8_t));
      break;
    }
    case CAN_MSG_RC_STATE_ID: {
      msg->type = AgxMsgRcState;
      memcpy(msg->body.rc_state_msg.raw, rx_frame->data,
             rx_frame->can_dlc * sizeof(uint8_t));
      break;
    }
    case CAN_MSG_CTRL_MODE_SELECT_ID: {
      msg->type = AgxMsgCtrlModeSelect;
      memcpy(msg->body.ctrl_mode_select_msg.raw, rx_frame->data,
             rx_frame->can_dlc * sizeof(uint8_t));
      break;
    }
    case CAN_MSG_STATE_RESET_ID: {
      msg->type = AgxMsgFaultByteReset;
      memcpy(msg->body.state_reset_msg.raw, rx_frame->data,
             rx_frame->can_dlc * sizeof(uint8_t));
      break;
    }
    // state feedback frame
    case CAN_MSG_SYSTEM_STATE_ID: {
      msg->type = AgxMsgSystemState;
      memcpy(msg->body.system_state_msg.raw, rx_frame->data,
             rx_frame->can_dlc * sizeof(uint8_t));
      break;
    }
    case CAN_MSG_MOTION_STATE_ID: {
      msg->type = AgxMsgMotionState;
      memcpy(msg->body.motion_state_msg.raw, rx_frame->data,
             rx_frame->can_dlc * sizeof(uint8_t));
      break;
    }
    case CAN_MSG_LIGHT_STATE_ID: {
      msg->type = AgxMsgLightState;
      memcpy(msg->body.light_state_msg.raw, rx_frame->data,
             rx_frame->can_dlc * sizeof(uint8_t));
      break;
    }
    case CAN_MSG_ODOMETRY_ID: {
      msg->type = AgxMsgOdometry;
      memcpy(msg->body.odometry_msg.raw, rx_frame->data,
             rx_frame->can_dlc * sizeof(uint8_t));
      break;
    }
    case CAN_MSG_ACTUATOR1_HS_STATE_ID:
    case CAN_MSG_ACTUATOR2_HS_STATE_ID:
    case CAN_MSG_ACTUATOR3_HS_STATE_ID:
    case CAN_MSG_ACTUATOR4_HS_STATE_ID: {
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
    case CAN_MSG_ACTUATOR4_LS_STATE_ID: {
      msg->type = AgxMsgActuatorLSState;
      msg->body.actuator_ls_state_msg.motor_id =
          (uint8_t)(rx_frame->can_id - CAN_MSG_ACTUATOR1_LS_STATE_ID);
      memcpy(msg->body.actuator_ls_state_msg.data.raw, rx_frame->data,
             rx_frame->can_dlc * sizeof(uint8_t));
      break;
    }
    case CAN_MSG_BMS_DATE_ID: {
      msg->type = AgxMsgBmsDate;
      memcpy(msg->body.bms_date_msg.raw, rx_frame->data,
             rx_frame->can_dlc * sizeof(uint8_t));
      break;
    }
    case CAN_MSG_BMS_STATUES_ID: {
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

