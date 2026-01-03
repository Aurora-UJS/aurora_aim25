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

#ifndef RM_SERIAL_DRIVER__PROTOCOL__PACKET_TOOL_HPP_
#define RM_SERIAL_DRIVER__PROTOCOL__PACKET_TOOL_HPP_

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <unordered_map>

#include "rm_serial_driver/protocol/crc.hpp"
#include "rm_serial_driver/protocol/packet_typedef.hpp"
#include "rm_serial_driver/transporter_interface.hpp"
#include "rm_utils/logger/log.hpp"

namespace aurora::serial_driver {

/********************************************************/
/* Packet Size Registry                                 */
/********************************************************/

// 根据包ID获取包大小
inline size_t getPacketSize(uint8_t packet_id) {
  static const std::unordered_map<uint8_t, size_t> packet_sizes = {
    {ID_IMU, sizeof(ReceiveImuData)},
    {ID_ROBOT_STATUS, sizeof(ReceiveRobotStatus)},
    {ID_SHOOT_DATA, sizeof(ReceiveShootData)},
    {ID_GAME_STATUS, sizeof(ReceiveGameStatus)},
    {ID_ALL_ROBOT_HP, sizeof(ReceiveAllRobotHp)},
    {ID_EVENT_DATA, sizeof(ReceiveEventData)},
    {ID_ROBOT_POSITION, sizeof(ReceiveRobotPosition)},
    {ID_RFID_STATUS, sizeof(ReceiveRfidStatus)},
    {ID_BUFF, sizeof(ReceiveBuff)},
    {ID_JOINT_STATE, sizeof(ReceiveJointState)},
  };

  auto it = packet_sizes.find(packet_id);
  if (it != packet_sizes.end()) {
    return it->second;
  }
  return 0;
}

/********************************************************/
/* Packet Handler                                       */
/********************************************************/

using PacketHandler = std::function<void(uint8_t id, const uint8_t *data, size_t len)>;

class PacketRouter {
public:
  explicit PacketRouter(std::shared_ptr<TransporterInterface> transporter)
  : transporter_(transporter) {
    AURORA_REGISTER_LOGGER("serial_driver", "~/aurora-log", INFO);
  }

  // 注册数据包处理器
  void registerHandler(uint8_t packet_id, PacketHandler handler) {
    handlers_[packet_id] = std::move(handler);
  }

  // 分发数据包
  void dispatch(uint8_t packet_id, const uint8_t *data, size_t len) {
    auto it = handlers_.find(packet_id);
    if (it != handlers_.end()) {
      it->second(packet_id, data, len);
    } else {
      AURORA_WARN("serial_driver", "No handler for packet ID: 0x{:02X}", packet_id);
    }
  }

  // 发送数据包
  template <typename PacketType>
  bool send(PacketType &packet) {
    // 填充帧头和帧尾
    packet.frame_header.sof = FRAME_HEADER;
    packet.frame_header.len = sizeof(typename std::remove_reference<decltype(packet.data)>::type);
    packet.tail = FRAME_TAIL;

    // 设置时间戳
    auto now = std::chrono::steady_clock::now();
    auto duration = now.time_since_epoch();
    packet.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();

    // 计算并填充CRC
    fillPacketCrc(packet);

    // 发送数据
    int sent = transporter_->write(&packet, sizeof(PacketType));
    if (sent != static_cast<int>(sizeof(PacketType))) {
      AURORA_ERROR("serial_driver", "Failed to send packet, sent {} bytes", sent);
      return false;
    }
    return true;
  }

  // 接收并解析数据包
  bool receiveAndDispatch() {
    uint8_t buffer[128];

    // 读取帧头
    if (transporter_->read(buffer, sizeof(HeaderFrame)) != sizeof(HeaderFrame)) {
      return false;
    }

    // 验证帧起始字节
    if (buffer[0] != FRAME_HEADER) {
      AURORA_WARN("serial_driver", "Invalid frame header: 0x{:02X}", buffer[0]);
      return false;
    }

    // 验证帧头CRC
    HeaderFrame *header = reinterpret_cast<HeaderFrame *>(buffer);
    if (!crc8_verify(buffer, 3, header->crc)) {
      AURORA_WARN("serial_driver", "Header CRC8 verification failed");
      return false;
    }

    // 获取数据包大小
    size_t packet_size = getPacketSize(header->id);
    if (packet_size == 0) {
      AURORA_WARN("serial_driver", "Unknown packet ID: 0x{:02X}", header->id);
      return false;
    }

    // 读取剩余数据
    size_t remaining = packet_size - sizeof(HeaderFrame);
    if (transporter_->read(buffer + sizeof(HeaderFrame), remaining) !=
        static_cast<int>(remaining)) {
      AURORA_WARN("serial_driver", "Failed to read packet body");
      return false;
    }

    // 验证帧尾
    if (buffer[packet_size - 1] != FRAME_TAIL) {
      AURORA_WARN("serial_driver", "Invalid frame tail: 0x{:02X}", buffer[packet_size - 1]);
      return false;
    }

    // 验证数据CRC16
    size_t crc_offset = packet_size - 3;  // crc(2) + tail(1)
    uint16_t received_crc;
    std::memcpy(&received_crc, buffer + crc_offset, sizeof(uint16_t));
    if (!crc16_verify(buffer, crc_offset, received_crc)) {
      AURORA_WARN("serial_driver", "Data CRC16 verification failed");
      return false;
    }

    // 分发数据包
    dispatch(header->id, buffer, packet_size);
    return true;
  }

private:
  std::shared_ptr<TransporterInterface> transporter_;
  std::unordered_map<uint8_t, PacketHandler> handlers_;
};

/********************************************************/
/* Packet Builder Helper                                */
/********************************************************/

template <typename PacketType>
class PacketBuilder {
public:
  PacketBuilder() {
    std::memset(&packet_, 0, sizeof(PacketType));
    packet_.frame_header.sof = FRAME_HEADER;
    packet_.tail = FRAME_TAIL;
  }

  PacketBuilder &setId(uint8_t id) {
    packet_.frame_header.id = id;
    return *this;
  }

  template <typename DataType>
  PacketBuilder &setData(const DataType &data) {
    static_assert(sizeof(DataType) == sizeof(packet_.data), "Data size mismatch");
    std::memcpy(&packet_.data, &data, sizeof(DataType));
    return *this;
  }

  PacketBuilder &stampNow() {
    auto now = std::chrono::steady_clock::now();
    auto duration = now.time_since_epoch();
    packet_.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
    return *this;
  }

  PacketType &build() {
    packet_.frame_header.len = sizeof(packet_.data);
    fillPacketCrc(packet_);
    return packet_;
  }

  const PacketType &get() const { return packet_; }

private:
  PacketType packet_;
};

}  // namespace aurora::serial_driver

#endif  // RM_SERIAL_DRIVER__PROTOCOL__PACKET_TOOL_HPP_
