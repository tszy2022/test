#ifndef CONNECTOR_H
#define CONNECTOR_H

#include <stdint.h>

#define ACTUATOR1_ID ((uint8_t)0x00)
#define ACTUATOR2_ID ((uint8_t)0x01)
#define ACTUATOR3_ID ((uint8_t)0x02)
#define ACTUATOR4_ID ((uint8_t)0x03)

/*--------------------------- Message IDs ------------------------------*/

#define CAN_MSG_MOTION_COMMAND_ID ((uint32_t)0x111)
#define CAN_MSG_LIGHT_COMMAND_ID ((uint32_t)0x121)
#define CAN_MSG_PARK_COMMAND_ID ((uint32_t)0x131)

#define CAN_MSG_SYSTEM_STATE_ID ((uint32_t)0x211)
#define CAN_MSG_MOTION_STATE_ID ((uint32_t)0x221)
#define CAN_MSG_LIGHT_STATE_ID ((uint32_t)0x231)
#define CAN_MSG_RC_STATE_ID ((uint32_t)0x241)

#define CAN_MSG_ACTUATOR1_HS_STATE_ID ((uint32_t)0x251)
#define CAN_MSG_ACTUATOR2_HS_STATE_ID ((uint32_t)0x252)
#define CAN_MSG_ACTUATOR3_HS_STATE_ID ((uint32_t)0x253)
#define CAN_MSG_ACTUATOR4_HS_STATE_ID ((uint32_t)0x254)

#define CAN_MSG_ACTUATOR1_LS_STATE_ID ((uint32_t)0x261)
#define CAN_MSG_ACTUATOR2_LS_STATE_ID ((uint32_t)0x262)
#define CAN_MSG_ACTUATOR3_LS_STATE_ID ((uint32_t)0x263)
#define CAN_MSG_ACTUATOR4_LS_STATE_ID ((uint32_t)0x264)

#define CAN_MSG_ODOMETRY_ID ((uint32_t)0x311)

#define CAN_MSG_BMS_DATE_ID ((uint32_t)0x361)
#define CAN_MSG_BMS_STATUES_ID ((uint32_t)0x362)

#define CAN_MSG_VERSION_QUERY_ID ((uint32_t)0x411)
#define CAN_MSG_PLATFORM_VERSION_ID ((uint32_t)0x41a)

#define CAN_MSG_CTRL_MODE_SELECT_ID ((uint32_t)0x421)
#define CAN_MSG_STEER_NEUTRAL_RESET_ID ((uint32_t)0x431)
#define CAN_MSG_STEER_NEUTRAL_RESET_ACK_ID ((uint32_t)0x43a)
#define CAN_MSG_STATE_RESET_ID ((uint32_t)0x441)

/*--------------------- Control/State Constants ------------------------*/

// Control
#define VEHICLE_STATE_NORMAL ((uint8_t)0x00)
#define VEHICLE_STATE_ESTOP ((uint8_t)0x01)
#define VEHICLE_STATE_EXCEPTION ((uint8_t)0x02)

#define FAULT_BATTERY_LOW_ERROR ((uint8_t)0x01)
#define FAULT_BATTERY_LOW_WARN ((uint8_t)0x02)
#define FAULT_RC_SIGNAL_LOSS ((uint8_t)0x04)

#define FAULT_CLR_ALL ((uint8_t)0x00)
#define FAULT_CLR_MOTOR1_COMM ((uint8_t)0x01)
#define FAULT_CLR_MOTOR2_COMM ((uint8_t)0x02)
#define FAULT_CLR_MOTOR3_COMM ((uint8_t)0x03)
#define FAULT_CLR_MOTOR4_COMM ((uint8_t)0x04)

#define CTRL_MODE_RC ((uint8_t)0x00)
#define CTRL_MODE_CMD_CAN ((uint8_t)0x01)
#define CTRL_MODE_CMD_UART ((uint8_t)0x02)

#define QUERY_PLATFORM_VERSION_REQUEST ((uint8_t)0x01)

// Actuator
#define BATTERY_VOLTAGE_LOW ((uint8_t)0x01)
#define MOTOR_OVERHEAT ((uint8_t)0x02)
#define MOTOR_DRIVER_OVERLOAD ((uint8_t)0x04)
#define MOTOR_DRIVER_OVERHEAT ((uint8_t)0x08)
#define MOTOR_SENSOR_FAULT ((uint8_t)0x10)
#define MOTOR_DRIVER_FAULT ((uint8_t)0x20)
#define MOTOR_DRIVER_ENABLED ((uint8_t)0x40)
#define MOTOR_DRIVER_RESERVED0 ((uint8_t)0x80)

#define STEER_NEUTRAL_RESET_ACK ((uint8_t)0xee)

// Light
#define LIGHT_CTRL_DISABLE ((uint8_t)0x00)
#define LIGHT_CTRL_ENABLE ((uint8_t)0x01)

#define LIGHT_MODE_CONST_OFF ((uint8_t)0x00)
#define LIGHT_MODE_CONST_ON ((uint8_t)0x01)
#define LIGHT_MODE_BREATH ((uint8_t)0x02)
#define LIGHT_MODE_CUSTOM ((uint8_t)0x03)
#pragma pack(push, 1)
typedef union {
  struct {
    struct {
      int8_t high_byte;
      int8_t low_byte;
    } linear_velocity;
    struct {
      uint8_t high_byte;
      uint8_t low_byte;
    } angular_velocity;
    struct {
      uint8_t high_byte;
      uint8_t low_byte;
    } lateral_velocity;
    struct {
      uint8_t high_byte;
      uint8_t low_byte;
    } steering_angle;
  } cmd;
  uint8_t raw[8];
} MotionCommandMessage;

typedef union {
  struct {
    uint8_t light_ctrl_enabled;
    uint8_t front_light_mode;
    uint8_t front_light_custom;
    uint8_t rear_light_mode;
    uint8_t rear_light_custom;
    uint8_t reserved0;
    uint8_t reserved1;
    uint8_t count;

  } cmd;
  uint8_t raw[8];
} LightCommandMessage;

typedef union {
  struct {
    uint8_t control_mode;
    uint8_t reserved0;
    uint8_t reserved1;
    uint8_t reserved2;
    uint8_t reserved3;
    uint8_t reserved4;
    uint8_t reserved5;
    uint8_t reserved6;
  } cmd;
  uint8_t raw[8];
} CtrlModeSelectMessage;

typedef union {
  struct {
    uint8_t parking_mode;
    uint8_t reserved0;
    uint8_t reserved1;
    uint8_t reserved2;
    uint8_t reserved3;
    uint8_t reserved4;
    uint8_t reserved5;
    uint8_t reserved6;
  } cmd;
  uint8_t raw[8];
} ParkControlMessage;


typedef union {
  struct {
    uint8_t fault_byte;
    uint8_t reserved0;
    uint8_t reserved1;
    uint8_t reserved2;
    uint8_t reserved3;
    uint8_t reserved4;
    uint8_t reserved5;
    uint8_t reserved6;
  } cmd;
  uint8_t raw[8];
} StateResetMessage;

// State feedback messages
typedef union {
  struct {
    uint8_t vehicle_state;
    uint8_t control_mode;
    struct {
      uint8_t high_byte;
      uint8_t low_byte;
    } battery_voltage;
    uint8_t fault_code;
    uint8_t reserved0;
    uint8_t park_mode;
    uint8_t count;
  } state;
  uint8_t raw[8];
} SystemStateMessage;

typedef union {
  struct {
    struct {
      uint8_t high_byte;
      uint8_t low_byte;
    } linear_velocity;
    struct {
      uint8_t high_byte;
      uint8_t low_byte;
    } angular_velocity;  // only valid for differential drivering
    uint8_t reserved0;
    uint8_t reserved1;
    struct {
      uint8_t high_byte;
      uint8_t low_byte;
    } steering_angle;  // only valid for ackermann steering
  } state;
  uint8_t raw[8];
} MotionStateMessage;

typedef union {
  struct {
    uint8_t light_ctrl_enabled;
    uint8_t front_light_mode;
    uint8_t front_light_custom;
    uint8_t rear_light_mode;
    uint8_t rear_light_custom;
    uint8_t reserved0;
    uint8_t reserved1;
    uint8_t count;
  } state;
  uint8_t raw[8];
} LightStateMessage;

typedef union {
  struct {
    uint8_t sws;
    uint8_t right_stick_left_right;
    uint8_t right_stick_up_down;
    uint8_t left_stick_left_right;
    uint8_t left_stick_up_down;
    uint8_t var_a;
    uint8_t reserved0;
    uint8_t count;
  } state;
  uint8_t raw[8];
} RcStateMessage;

typedef struct {
  uint8_t motor_id;
  union {
    struct {
      struct {
        uint8_t high_byte;
        uint8_t low_byte;
      } rpm;
      struct {
        uint8_t high_byte;
        uint8_t low_byte;
      } current;
      struct {
        int8_t msb;
        int8_t high_byte;
        int8_t low_byte;
        int8_t lsb;
      } pulse_count;
    } state;
    uint8_t raw[8];
  } data;
} ActuatorHSStateMessage;

typedef struct {
  uint8_t motor_id;
  union {
    struct {
      struct {
        uint8_t high_byte;
        uint8_t low_byte;
      } driver_voltage;
      struct {
        uint8_t high_byte;
        uint8_t low_byte;
      } driver_temperature;
      int8_t motor_temperature;
      uint8_t driver_state;
      uint8_t reserved0;
      uint8_t reserved1;
    } state;
    uint8_t raw[8];
  } data;
} ActuatorLSStateMessage;

typedef union {
  struct {
    struct {
      uint8_t msb;
      uint8_t high_byte;
      uint8_t low_byte;
      uint8_t lsb;
    } left_wheel;
    struct {
      uint8_t msb;
      uint8_t high_byte;
      uint8_t low_byte;
      uint8_t lsb;
    } right_wheel;
  } state;
  uint8_t raw[8];
} OdometryMessage;

typedef union {
  struct {
    uint8_t battery_SOC;
    uint8_t battery_SOH;
    struct {
      uint8_t high_byte;
      uint8_t low_byte;
    } battery_voltage;
    struct {
      uint8_t high_byte;
      uint8_t low_byte;
    } battery_current;
    struct {
      uint8_t high_byte;
      uint8_t low_byte;
    } battery_temperature;
  } state;
  uint8_t raw[8];
} BMSDateMessage;

typedef union {
  struct {
    uint8_t Alarm_Status_1;
    uint8_t Alarm_Status_2;
    uint8_t Warning_Status_1;
    uint8_t Warning_Status_2;
    uint8_t reserved0;
    uint8_t reserved1;
    uint8_t reserved2;
    uint8_t reserved3;
  } state;
  uint8_t raw[8];
} BMSStatusMessage;

typedef union {
  struct {
    uint8_t request;
    uint8_t reserved0;
    uint8_t reserved1;
    uint8_t reserved2;
    uint8_t reserved3;
    uint8_t reserved4;
    uint8_t reserved5;
    uint8_t reserved6;
  } state;
  uint8_t raw[8];
} VersionQueryMessage;

typedef union {
  struct {
    struct {
      uint8_t high_byte;
      uint8_t low_byte;
    } controller_hw_version;
    struct {
      uint8_t high_byte;
      uint8_t low_byte;
    } motor_driver_hw_version;
    struct {
      uint8_t high_byte;
      uint8_t low_byte;
    } controller_sw_version;
    struct {
      uint8_t high_byte;
      uint8_t low_byte;
    } motor_driver_sw_version;
  } state;
  uint8_t raw[8];
} PlatformVersionMessage;

#pragma pack(pop)

struct ScoutState {
  enum MotorID {
    FRONT_RIGHT = 0,
    FRONT_LEFT = 1,
    REAR_LEFT = 2,
    REAR_RIGHT = 3
  };

  struct ActuatorState {
    double motor_current = 0;  // in A
    double motor_rpm = 0;
    uint16_t motor_pulses = 0;
    double motor_temperature = 0;

    double driver_voltage = 0;
    double driver_temperature = 0;
    uint8_t driver_state = 0;
  };

  struct LightState {
    uint8_t mode = 0;
    uint8_t custom_value = 0;
  };

  // base state
  uint8_t base_state = 0;
  uint8_t control_mode = 0;
  uint8_t fault_code = 0;
  double battery_voltage = 0.0;

  // motor state
  static constexpr uint8_t motor_num = 4;
  ActuatorState actuator_states[motor_num];

  // light state
  bool light_control_enabled = false;
  LightState front_light_state;
  LightState rear_light_state;

  // motion state
  double linear_velocity = 0;
  double angular_velocity = 0;

  // odometer state
  double left_odometry = 0;
  double right_odometry = 0;

  // BMS date
  uint8_t SOC;
  uint8_t SOH;
  double bms_battery_voltage = 0.0;
  double battery_current = 0.0;
  double battery_temperature = 0.0;

  // BMS state
  uint8_t Alarm_Status_1;
  uint8_t Alarm_Status_2;
  uint8_t Warning_Status_1;
  uint8_t Warning_Status_2;

};
struct can_frame {//定义can结构
            uint32_t can_id;
            uint8_t can_dlc=8;
            uint8_t data[8] __attribute__((aligned(8)));
                };

#pragma pack(push, 1)
typedef enum {//使用枚举变量定义通信信息种类
  AgxMsgUnkonwn = 0x00,
  // command
  AgxMsgMotionCommand = 0x01,
  AgxMsgLightCommand = 0x02,
  AgxMsgCtrlModeSelect = 0x03,
  AgxMsgFaultByteReset = 0x04,
  AgxMsgParkModeSelect = 0x05,
  // state feedback
  AgxMsgSystemState = 0x21,
  AgxMsgMotionState = 0x22,
  AgxMsgLightState = 0x23,
  AgxMsgRcState = 0x24,
  AgxMsgActuatorHSState = 0x25,
  AgxMsgActuatorLSState = 0x26,
  AgxMsgOdometry = 0x27,
  AgxMsgVersionQuery = 0x28,
  AgxMsgPlatformVersion = 0x29,
  AgxMsgBmsDate = 0x30,
  AgxMsgBmsStatus = 0x31
} MsgType;

typedef struct {
  MsgType type;
  union {//使用联合控制某一时间只有一个信息有值
    // command
    MotionCommandMessage motion_command_msg;
    LightCommandMessage light_command_msg;
    CtrlModeSelectMessage ctrl_mode_select_msg;
    StateResetMessage state_reset_msg;
    ParkControlMessage park_control_msg;
    // state feedback
    SystemStateMessage system_state_msg;
    MotionStateMessage motion_state_msg;
    LightStateMessage light_state_msg;
    RcStateMessage rc_state_msg;
    ActuatorHSStateMessage actuator_hs_state_msg;
    ActuatorLSStateMessage actuator_ls_state_msg;
    OdometryMessage odometry_msg;
    VersionQueryMessage version_query_msg;
    PlatformVersionMessage platform_version_msg;
    BMSDateMessage bms_date_msg;
    BMSStatusMessage bms_status_msg;

  } body;
} AgxMessage;
#pragma pack(pop)

struct ScoutLightCmd {
  enum class LightMode {
    CONST_OFF = 0x00,
    CONST_ON = 0x01,
    BREATH = 0x02,
    CUSTOM = 0x03
  };

  ScoutLightCmd() = default;
  ScoutLightCmd(LightMode f_mode, uint8_t f_value, LightMode r_mode,
                uint8_t r_value)
      : enable_ctrl(true),
        front_mode(f_mode),
        front_custom_value(f_value),
        rear_mode(r_mode),
        rear_custom_value(r_value) {}

  bool enable_ctrl = false;
  LightMode front_mode;
  uint8_t front_custom_value;
  LightMode rear_mode;
  uint8_t rear_custom_value;
};

struct ScoutMotionCmd {
  enum class FaultClearFlag
  {
      NO_FAULT = 0x00,
      BAT_UNDER_VOL = 0x01,
      BAT_OVER_VOL = 0x02,
      MOTOR1_COMM = 0x03,
      MOTOR2_COMM = 0x04,
      MOTOR3_COMM = 0x05,
      MOTOR4_COMM = 0x06,
      MOTOR_DRV_OVERHEAT = 0x07,
      MOTOR_OVERCURRENT = 0x08
  };

  ScoutMotionCmd(double linear = 0.0, double angular = 0.0,
                 FaultClearFlag fault_clr_flag = FaultClearFlag::NO_FAULT)
      : linear_velocity(linear), angular_velocity(angular),
        fault_clear_flag(fault_clr_flag){}

  double linear_velocity;
  double angular_velocity;
  double lateral_velocity;
  FaultClearFlag fault_clear_flag;

};
struct ScoutCmdLimits {
  static constexpr double max_linear_velocity = 1.5;       // 1.5 m/s
  static constexpr double min_linear_velocity = -1.5;      // -1.5 m/s
  static constexpr double max_angular_velocity = 0.5235;   // 0.5235 rad/s
  static constexpr double min_angular_velocity = -0.5235;  // -0.5235 rad/s
};

class Connector
{
    public:
        int m_sockfd;
        unsigned char rec_buffer[13] {};
        uint8_t rec_buffer0[13] {};
        can_frame canframe;
        can_frame* can_frame_pt{ &canframe };
        AgxMessage agx_msg;
		AgxMessage* agx_msg_pt{ &agx_msg };
		ScoutState scout_state;
		ScoutState* scout_state_pt{ &scout_state };
		ScoutLightCmd scout_light_cmd;
		ScoutMotionCmd scout_motion_cmd;
		ScoutCmdLimits scout_cmd_limits;
        Connector();
        int init();
		bool DecodeCanFrame(const struct can_frame *rx_frame, AgxMessage *msg);

       void unpack_all();
        void convert_data_once(const AgxMessage &status_msg,ScoutState &state);
        void print();
        void printall();
         bool ConnectToServer(const char *serverip,const int port);
  // 向对端发送报文
        int  Send(const void *buf,const int buflen);
  // 接收对端的报文
        int  Recv(void *buf,const int buflen);
  //桌面
        int Read(void *buf,const int buflen);
        void copy_to_can_frame(can_frame *rx_frame, uint8_t *msg);
        void copy_to_buffer(can_frame *rx_frame);
        void cmd_test();
        void SetMotionCommand();
        void SendLightCmd(const ScoutLightCmd &lcmd, uint8_t count);
        void SetLightCommand(const ScoutLightCmd &cmd);
		void EncodeCanFrame(const AgxMessage *msg, struct can_frame *tx_frame);
        ~Connector();
    protected:

    private:
};

#endif // CONNECTOR_H
