#pragma once

#include <cstdint>
#include <variant>

constexpr uint8_t FRAME_HEADER_BYTE_0 = 0x55;
constexpr uint8_t FRAME_HEADER_BYTE_1 = 0xAA;
constexpr uint8_t FRAME_HEADER_BYTE_2 = 0x05;
constexpr uint8_t FRAME_HEADER_BYTE_3 = 0x0A;

constexpr uint8_t FRAME_TAIL_BYTE_0 = 0x00;
constexpr uint8_t FRAME_TAIL_BYTE_1 = 0xFF;

const uint32_t ACK_DATA_PACKET_TYPE = 101;
const uint32_t POINT_DATA_PACKET_TYPE = 102;

const uint32_t IMU_DATA_PACKET_TYPE = 104;
const uint32_t TIME_STAMP_PACKET_TYPE = 106;

const uint32_t COMMAND_PACKET_TYPE = 2000;
const uint32_t WORK_MODE_CONFIG_PACKET_TYPE = 2002;

const uint32_t CMD_RESET_TYPE = 1;
const uint32_t CMD_PARAM_SAVE = 2;
const uint32_t CMD_PARAM_GET = 3;
const uint32_t CMD_VERSION_GET = 4;
const uint32_t CMD_STANDBY_TYPE = 5;
const uint32_t CMD_LATENCY_TYPE = 6;
const uint32_t CMD_CONFIG_RESET = 7;

const uint32_t USER_CMD_RESET_TYPE = 1;
const uint32_t USER_CMD_STANDBY_TYPE = 2;
const uint32_t USER_CMD_VERSION_GET = 3;
const uint32_t USER_CMD_LATENCY_TYPE = 4;
const uint32_t USER_CMD_CONFIG_RESET = 5;
const uint32_t USER_CMD_CONFIG_GET = 6;
const uint32_t USER_CMD_CONFIG_AUTO_STANDBY = 7;

const uint32_t ACK_SUCCESS = 1;
const uint32_t ACK_CRC_ERROR = 2;
const uint32_t ACK_HEADER_ERROR = 3;
const uint32_t ACK_BLOCK_ERROR = 4;
const uint32_t ACK_WAIT_ERROR = 5;

struct FrameHeader
{
    uint8_t header[4];
    uint32_t packet_type;
    uint32_t packet_size;
};

struct FrameTail
{
    uint32_t crc32;
    uint32_t msg_type_check;
    uint8_t reserve[2];
    uint8_t tail[2];
};

struct TimeStamp
{
    uint32_t sec;
    uint32_t nsec;
};

struct DataInfo
{
    uint32_t seq;
    uint32_t payload_size;
    TimeStamp stamp;
};

struct CalibParam
{
    float a_axis_dist;      // meters
    float b_axis_dist;      // meters
    float theta_angle_bias; // radians
    float alpha_angle_bias; // radians
    float beta_angle;       // radians
    float xi_angle;         // radians
    float range_bias;       // meters
    float range_scale;      // unitless
};

struct InsideState
{
    uint32_t sys_rotation_period; // Vertical motor (microseconds)
    uint32_t com_rotation_period; // Horizontal motor (microseconds)
    float dirty_index;
    float packet_lost_up;
    float packet_lost_down;
    float apd_temperature;
    float apd_voltage;
    float laser_voltage;
    float imu_temperature;
};

struct PointData
{
    DataInfo info;
    InsideState state;
    CalibParam param;

    float com_horizontal_angle_start; // radians
    float com_horizontal_angle_step;  // radians
    float scan_period;                // seconds
    float range_min;                  // meters
    float range_max;                  // meters
    float angle_min;                  // radians
    float angle_increment;            // radians
    float time_increment;             // seconds
    uint32_t point_num;
    uint16_t ranges[300]; // millimeters
    uint8_t intensities[300];
};

struct PointDataPacket
{
    FrameHeader header;
    PointData data;
    FrameTail tail;
};

struct TimeStampPacket
{
    FrameHeader header;
    TimeStamp data;
    FrameTail tail;
};

struct ImuData
{
    DataInfo info;
    float quaternion[4];
    float angular_velocity[3];
    float linear_acceleration[3];
};

struct ImuDataPacket
{
    FrameHeader header;
    ImuData data;
    FrameTail tail;
};

struct AckData
{
    uint32_t packet_type; // Type of current packet
    uint32_t cmd_type;    // Type of packet being acknowledged
    uint32_t cmd_value;   // Value of packet being acknowledged
    uint32_t status;      // Result of command execution
};

struct AckDataPacket
{
    FrameHeader header;
    AckData data;
    FrameTail tail;
};

struct VersionData
{
    uint8_t hw_version[4];
    uint8_t sw_version[4];
    uint8_t name[24];
    uint8_t date[8];
    uint8_t reserve[40];
};

struct VersionDataPacket
{
    FrameHeader header;
    VersionData data;
    FrameTail tail;
};

struct IpAddressConfig
{
    uint8_t lidar_ip[4]; // The lidar's local IP address
    uint8_t user_ip[4];  // The user's remote IP address
    uint8_t gateway[4];
    uint8_t subnet_mask[4];
    uint16_t lidar_port; // The lidar's local port
    uint16_t user_port;  // The user's remote port
};

struct IpAddressConfigPacket
{
    FrameHeader header;
    IpAddressConfig data;
    FrameTail tail;
};

struct WorkModeConfig
{
    uint32_t mode;
};

struct WorkModeConfigPacket
{
    FrameHeader header;
    WorkModeConfig data;
    FrameTail tail;
};

struct UserCtrlCmd
{
    uint32_t type;
    uint32_t value;
};

struct UserCtrlCmdPacket
{
    FrameHeader header;
    UserCtrlCmd data;
    FrameTail tail;
};

// using LidarMessage = std::variant<PointDataPacket, ImuDataPacket, AckDataPacket, VersionDataPacket, IPAddressConfigPacket, WorkModeConfigPacket, UserCtrlCmdPacket>;
using LidarMessage = std::variant<PointDataPacket, ImuDataPacket>;