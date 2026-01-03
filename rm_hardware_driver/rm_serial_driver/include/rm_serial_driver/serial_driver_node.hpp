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

#ifndef RM_SERIAL_DRIVER__SERIAL_DRIVER_NODE_HPP_
#define RM_SERIAL_DRIVER__SERIAL_DRIVER_NODE_HPP_

// std
#include <atomic>
#include <memory>
#include <string>
#include <thread>
// ros2
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
// interfaces
#include <rm_interfaces/msg/gimbal_cmd.hpp>
#include <rm_interfaces/msg/serial_receive_data.hpp>
#include <rm_interfaces/srv/set_mode.hpp>
// project
#include "rm_serial_driver/protocol/crc.hpp"
#include "rm_serial_driver/protocol/packet_tool.hpp"
#include "rm_serial_driver/protocol/packet_typedef.hpp"
#include "rm_serial_driver/transporter_interface.hpp"
#include "rm_utils/heartbeat.hpp"

namespace aurora::serial_driver {

class SerialDriverNode : public rclcpp::Node {
public:
  explicit SerialDriverNode(const rclcpp::NodeOptions &options);
  ~SerialDriverNode() override;

private:
  // 初始化函数
  void init();
  void initTransporter();
  void initPublishers();
  void initSubscribers();
  void initPacketHandlers();

  // 主循环
  void listenLoop();

  // 数据包处理器
  void handleImuData(uint8_t id, const uint8_t *data, size_t len);

  // 回调函数
  void gimbalCmdCallback(const rm_interfaces::msg::GimbalCmd::SharedPtr msg);

  // TF广播
  void publishGimbalTF(float roll, float pitch, float yaw);

  // 设置模式客户端
  struct SetModeClient {
    rclcpp::Client<rm_interfaces::srv::SetMode>::SharedPtr ptr;
    std::atomic<uint8_t> mode{0};
    std::atomic<bool> on_waiting{false};
  };
  void setMode(SetModeClient &client, uint8_t mode);
  std::map<std::string, SetModeClient> set_mode_clients_;

  // 传输层
  std::shared_ptr<TransporterInterface> transporter_;
  std::unique_ptr<PacketRouter> packet_router_;

  // 发布者
  rclcpp::Publisher<rm_interfaces::msg::SerialReceiveData>::SharedPtr serial_receive_pub_;

  // 订阅者
  rclcpp::Subscription<rm_interfaces::msg::GimbalCmd>::SharedPtr gimbal_cmd_sub_;

  // TF广播器
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // 心跳
  aurora::HeartBeatPublisher::SharedPtr heartbeat_;

  // 监听线程
  std::unique_ptr<std::thread> listen_thread_;
  std::atomic<bool> running_{true};

  // 参数
  std::string target_frame_;
  std::string port_name_;
  double timestamp_offset_{0.0};

  // 状态缓存
  std::atomic<uint8_t> current_mode_{0};
  std::atomic<uint8_t> current_color_{0};
};

}  // namespace aurora::serial_driver

#endif  // RM_SERIAL_DRIVER__SERIAL_DRIVER_NODE_HPP_
