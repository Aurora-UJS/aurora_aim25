// Copyright (C) Aurora Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RM_SERIAL_DRIVER__PROTOCOL__PACKET_TYPEDEF_HPP_
#define RM_SERIAL_DRIVER__PROTOCOL__PACKET_TYPEDEF_HPP_

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace aurora::serial_driver {

/********************************************************/
/* Protocol Constants                                   */
/********************************************************/

constexpr uint8_t FRAME_HEADER = 0x5A;
constexpr uint8_t FRAME_TAIL = 0xA5;

// Receive Packet IDs
constexpr uint8_t ID_IMU = 0x01;
constexpr uint8_t ID_ROBOT_STATUS = 0x02;
constexpr uint8_t ID_SHOOT_DATA = 0x03;
constexpr uint8_t ID_GAME_STATUS = 0x04;
constexpr uint8_t ID_ALL_ROBOT_HP = 0x05;
constexpr uint8_t ID_EVENT_DATA = 0x06;
constexpr uint8_t ID_ROBOT_POSITION = 0x07;
constexpr uint8_t ID_RFID_STATUS = 0x08;
constexpr uint8_t ID_BUFF = 0x09;
constexpr uint8_t ID_JOINT_STATE = 0x0A;

// Send Packet IDs
constexpr uint8_t ID_GIMBAL_CMD = 0x01;
constexpr uint8_t ID_CHASSIS_CMD = 0x02;

/********************************************************/
/* Header Frame                                         */
/********************************************************/

struct HeaderFrame {
  uint8_t sof;  // 帧起始字节，固定值为 0x5A
  uint8_t len;  // 数据段长度（不包含帧头和帧尾）
  uint8_t id;   // 数据段ID
  uint8_t crc;  // 帧头的 CRC8 校验
} __attribute__((packed));

static_assert(sizeof(HeaderFrame) == 4, "HeaderFrame size mismatch");

/********************************************************/
/* Receive Packets                                      */
/********************************************************/

// IMU 数据包
struct ReceiveImuData {
  HeaderFrame frame_header;
  uint32_t timestamp;

  struct {
    float roll;   // rad
    float pitch;  // rad
    float yaw;    // rad

    float roll_vel;   // rad/s
    float pitch_vel;  // rad/s
    float yaw_vel;    // rad/s
  } __attribute__((packed)) data;

  uint16_t crc;
  uint8_t tail;
} __attribute__((packed));

static_assert(sizeof(ReceiveImuData) == 35, "ReceiveImuData size mismatch");

// 机器人状态数据包
struct ReceiveRobotStatus {
  HeaderFrame frame_header;
  uint32_t timestamp;

  struct {
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t current_hp;
    uint16_t maximum_hp;
    uint16_t shooter_cooling_value;
    uint16_t shooter_heat_limit;
    uint16_t shooter_17mm_heat;
    uint16_t projectile_allowance_17mm;
    float bullet_speed;    // 设定的弹速
    uint8_t target_color;  // 0: 蓝色, 1: 红色
    uint8_t mode;          // 下位机模式
  } __attribute__((packed)) data;

  uint16_t crc;
  uint8_t tail;
} __attribute__((packed));

static_assert(sizeof(ReceiveRobotStatus) == 31, "ReceiveRobotStatus size mismatch");

// 射击数据包
struct ReceiveShootData {
  HeaderFrame frame_header;
  uint32_t timestamp;

  struct {
    uint8_t bullet_type;     // 子弹类型
    uint8_t shooter_number;  // 发射机构ID
    uint8_t launching_freq;  // 射频
    float bullet_speed;      // 射速
  } __attribute__((packed)) data;

  uint16_t crc;
  uint8_t tail;
} __attribute__((packed));

static_assert(sizeof(ReceiveShootData) == 18, "ReceiveShootData size mismatch");

// 比赛状态数据包
struct ReceiveGameStatus {
  HeaderFrame frame_header;
  uint32_t timestamp;

  struct {
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
  } __attribute__((packed)) data;

  uint16_t crc;
  uint8_t tail;
} __attribute__((packed));

static_assert(sizeof(ReceiveGameStatus) == 14, "ReceiveGameStatus size mismatch");

// 全场机器人HP数据包
struct ReceiveAllRobotHp {
  HeaderFrame frame_header;
  uint32_t timestamp;

  struct {
    uint16_t red_1_robot_hp;
    uint16_t red_2_robot_hp;
    uint16_t red_3_robot_hp;
    uint16_t red_4_robot_hp;
    uint16_t red_5_robot_hp;
    uint16_t red_7_robot_hp;
    uint16_t red_outpost_hp;
    uint16_t red_base_hp;

    uint16_t blue_1_robot_hp;
    uint16_t blue_2_robot_hp;
    uint16_t blue_3_robot_hp;
    uint16_t blue_4_robot_hp;
    uint16_t blue_5_robot_hp;
    uint16_t blue_7_robot_hp;
    uint16_t blue_outpost_hp;
    uint16_t blue_base_hp;
  } __attribute__((packed)) data;

  uint16_t crc;
  uint8_t tail;
} __attribute__((packed));

static_assert(sizeof(ReceiveAllRobotHp) == 43, "ReceiveAllRobotHp size mismatch");

// 场地事件数据包
struct ReceiveEventData {
  HeaderFrame frame_header;
  uint32_t timestamp;

  struct {
    uint32_t supply_zone_non_overlapping : 1;
    uint32_t supply_zone_overlapping : 1;
    uint32_t supply_zone : 1;
    uint32_t small_energy : 1;
    uint32_t big_energy : 1;
    uint32_t central_highland : 2;
    uint32_t trapezoidal_highland : 2;
    uint32_t center_gain_zone : 2;
    uint32_t reserved : 21;
  } __attribute__((packed)) data;

  uint16_t crc;
  uint8_t tail;
} __attribute__((packed));

static_assert(sizeof(ReceiveEventData) == 15, "ReceiveEventData size mismatch");

// 机器人位置数据包
struct ReceiveRobotPosition {
  HeaderFrame frame_header;
  uint32_t timestamp;

  struct {
    float hero_x;
    float hero_y;
    float engineer_x;
    float engineer_y;
    float standard_3_x;
    float standard_3_y;
    float standard_4_x;
    float standard_4_y;
    float standard_5_x;
    float standard_5_y;
  } __attribute__((packed)) data;

  uint16_t crc;
  uint8_t tail;
} __attribute__((packed));

static_assert(sizeof(ReceiveRobotPosition) == 51, "ReceiveRobotPosition size mismatch");

// RFID状态数据包
struct ReceiveRfidStatus {
  HeaderFrame frame_header;
  uint32_t timestamp;

  struct {
    uint32_t base_gain_point : 1;
    uint32_t central_highland : 1;
    uint32_t enemy_central_highland : 1;
    uint32_t friendly_trapezoidal : 1;
    uint32_t enemy_trapezoidal : 1;
    uint32_t friendly_fly_ramp_front : 1;
    uint32_t friendly_fly_ramp_back : 1;
    uint32_t enemy_fly_ramp_front : 1;
    uint32_t enemy_fly_ramp_back : 1;
    uint32_t friendly_outpost : 1;
    uint32_t friendly_fortress : 1;
    uint32_t friendly_supply_zone : 1;
    uint32_t center_gain_point : 1;
    uint32_t reserved : 19;
  } __attribute__((packed)) data;

  uint16_t crc;
  uint8_t tail;
} __attribute__((packed));

static_assert(sizeof(ReceiveRfidStatus) == 15, "ReceiveRfidStatus size mismatch");

// 机器人增益数据包
struct ReceiveBuff {
  HeaderFrame frame_header;
  uint32_t timestamp;

  struct {
    uint8_t recovery_buff;
    uint8_t cooling_buff;
    uint8_t defence_buff;
    uint8_t vulnerability_buff;
    uint16_t attack_buff;
    uint8_t remaining_energy;
  } __attribute__((packed)) data;

  uint16_t crc;
  uint8_t tail;
} __attribute__((packed));

static_assert(sizeof(ReceiveBuff) == 18, "ReceiveBuff size mismatch");

// 云台关节状态数据包
struct ReceiveJointState {
  HeaderFrame frame_header;
  uint32_t timestamp;

  struct {
    float pitch;
    float yaw;
  } __attribute__((packed)) data;

  uint16_t crc;
  uint8_t tail;
} __attribute__((packed));

static_assert(sizeof(ReceiveJointState) == 19, "ReceiveJointState size mismatch");

/********************************************************/
/* Send Packets                                         */
/********************************************************/

// 云台控制数据包
struct SendGimbalCmd {
  HeaderFrame frame_header;
  uint32_t timestamp;

  struct {
    float pitch;     // rad
    float yaw;       // rad
    float distance;  // m
    uint8_t fire_advice;
    uint8_t tracking;
  } __attribute__((packed)) data;

  uint16_t crc;
  uint8_t tail;
} __attribute__((packed));

static_assert(sizeof(SendGimbalCmd) == 25, "SendGimbalCmd size mismatch");

// 底盘控制数据包
struct SendChassisCmd {
  HeaderFrame frame_header;
  uint32_t timestamp;

  struct {
    float vx;
    float vy;
    float wz;
  } __attribute__((packed)) data;

  uint16_t crc;
  uint8_t tail;
} __attribute__((packed));

static_assert(sizeof(SendChassisCmd) == 23, "SendChassisCmd size mismatch");

/********************************************************/
/* Template Utilities                                   */
/********************************************************/

template <typename T>
inline T fromVector(const std::vector<uint8_t> &data) {
  if (data.size() < sizeof(T)) {
    throw std::runtime_error("Data size too small for packet conversion");
  }
  T packet;
  std::memcpy(&packet, data.data(), sizeof(T));
  return packet;
}

template <typename T>
inline std::vector<uint8_t> toVector(const T &data) {
  std::vector<uint8_t> packet(sizeof(T));
  std::memcpy(packet.data(), &data, sizeof(T));
  return packet;
}

template <typename T>
inline void copyToBuffer(const T &data, uint8_t *buffer) {
  std::memcpy(buffer, &data, sizeof(T));
}

template <typename T>
inline T copyFromBuffer(const uint8_t *buffer) {
  T packet;
  std::memcpy(&packet, buffer, sizeof(T));
  return packet;
}

}  // namespace aurora::serial_driver

#endif  // RM_SERIAL_DRIVER__PROTOCOL__PACKET_TYPEDEF_HPP_
