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

#include "rm_serial_driver/serial_driver_node.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <Eigen/Geometry>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>

#include "rm_serial_driver/uart_transporter.hpp"
#include "rm_utils/logger/log.hpp"
#include "rm_utils/math/utils.hpp"

namespace aurora::serial_driver {

SerialDriverNode::SerialDriverNode(const rclcpp::NodeOptions &options)
: Node("serial_driver", options) {
  AURORA_REGISTER_LOGGER("serial_driver", "~/aurora2026-log", INFO);

  // 延迟初始化，因为构造函数中shared_from_this()不可用
  listen_thread_ = std::make_unique<std::thread>(&SerialDriverNode::listenLoop, this);
}

SerialDriverNode::~SerialDriverNode() {
  AURORA_INFO("serial_driver", "Destroy SerialDriverNode!");
  running_.store(false);
  if (listen_thread_ != nullptr && listen_thread_->joinable()) {
    listen_thread_->join();
  }
}

void SerialDriverNode::init() {
  AURORA_INFO("serial_driver", "Initializing SerialDriverNode with new protocol!");

  // 声明参数
  target_frame_ = this->declare_parameter("target_frame", "odom");
  port_name_ = this->declare_parameter("port_name", "/dev/ttyUSB0");
  timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);

  // 初始化各组件
  initTransporter();
  initPublishers();
  initSubscribers();
  initPacketHandlers();

  // TF广播器
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // 心跳
  heartbeat_ = aurora::HeartBeatPublisher::create(this);

  AURORA_INFO("serial_driver", "SerialDriverNode initialized with port: {}", port_name_);
}

void SerialDriverNode::initTransporter() {
  transporter_ = std::make_shared<UartTransporter>(port_name_);
  if (!transporter_->open()) {
    AURORA_ERROR("serial_driver", "Failed to open serial port: {}", port_name_);
  }
  packet_router_ = std::make_unique<PacketRouter>(transporter_);
}

void SerialDriverNode::initPublishers() {
  serial_receive_pub_ = this->create_publisher<rm_interfaces::msg::SerialReceiveData>(
    "serial/receive", rclcpp::SensorDataQoS());
}

void SerialDriverNode::initSubscribers() {
  gimbal_cmd_sub_ = this->create_subscription<rm_interfaces::msg::GimbalCmd>(
    "serial/gimbal_cmd",
    rclcpp::SensorDataQoS(),
    std::bind(&SerialDriverNode::gimbalCmdCallback, this, std::placeholders::_1));
}

void SerialDriverNode::initPacketHandlers() {
  using namespace std::placeholders;

  // 注册IMU数据包处理器
  packet_router_->registerHandler(ID_IMU,
                                  std::bind(&SerialDriverNode::handleImuData, this, _1, _2, _3));

  AURORA_INFO("serial_driver", "Packet handlers registered");
}

void SerialDriverNode::listenLoop() {
  // 延迟初始化
  init();

  while (running_.load() && rclcpp::ok()) {
    // 接收并分发数据包
    if (!packet_router_->receiveAndDispatch()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
}

void SerialDriverNode::handleImuData(uint8_t id, const uint8_t *data, size_t len) {
  (void)id;
  (void)len;

  auto packet = copyFromBuffer<ReceiveImuData>(data);

  // 提取数据
  float roll = packet.data.roll;
  float pitch = packet.data.pitch;
  float yaw = packet.data.yaw;

  // 发布ROS消息
  rm_interfaces::msg::SerialReceiveData msg;
  msg.header.stamp = this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
  msg.header.frame_id = target_frame_;
  msg.roll = roll;
  msg.pitch = pitch;
  msg.yaw = yaw;
  msg.mode = current_mode_.load();
  msg.bullet_speed = 0.0f;  // 从 robot_status 获取

  serial_receive_pub_->publish(msg);

  // 广播TF
  publishGimbalTF(roll, pitch, yaw);
}

void SerialDriverNode::gimbalCmdCallback(const rm_interfaces::msg::GimbalCmd::SharedPtr msg) {
  SendGimbalCmd packet;
  packet.frame_header.id = ID_GIMBAL_CMD;

  packet.data.pitch = static_cast<float>(msg->pitch);
  packet.data.yaw = static_cast<float>(msg->yaw);
  packet.data.distance = static_cast<float>(msg->distance);
  packet.data.fire_advice = msg->fire_advice ? 1 : 0;
  packet.data.tracking = 1;

  packet_router_->send(packet);
}

void SerialDriverNode::publishGimbalTF(float roll, float pitch, float yaw) {
  timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
  t.header.frame_id = target_frame_;
  t.child_frame_id = "gimbal_link";

  // 角度转换
  auto roll_rad = roll * M_PI / 180.0;
  auto pitch_rad = -pitch * M_PI / 180.0;
  auto yaw_rad = yaw * M_PI / 180.0;

  tf2::Quaternion q;
  q.setRPY(roll_rad, pitch_rad, yaw_rad);
  t.transform.rotation = tf2::toMsg(q);
  tf_broadcaster_->sendTransform(t);

  // odom_rectify: 转了roll角后的坐标系
  Eigen::Quaterniond q_eigen(q.w(), q.x(), q.y(), q.z());
  Eigen::Vector3d rpy = utils::getRPY(q_eigen.toRotationMatrix());
  q.setRPY(rpy[0], 0, 0);
  t.header.frame_id = target_frame_;
  t.child_frame_id = target_frame_ + "_rectify";
  tf_broadcaster_->sendTransform(t);
}

void SerialDriverNode::setMode(SetModeClient &client, uint8_t mode) {
  using namespace std::chrono_literals;

  std::string service_name = client.ptr->get_service_name();

  while (!client.ptr->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      AURORA_ERROR(
        "serial_driver", "Interrupted while waiting for service {}. Exiting.", service_name);
      return;
    }
    AURORA_INFO("serial_driver", "Service {} not available, waiting...", service_name);
  }

  if (!client.ptr->service_is_ready()) {
    AURORA_WARN("serial_driver", "Service {} is not available!", service_name);
    return;
  }

  auto req = std::make_shared<rm_interfaces::srv::SetMode::Request>();
  req->mode = mode;

  client.on_waiting.store(true);
  auto result = client.ptr->async_send_request(
    req, [mode, &client](rclcpp::Client<rm_interfaces::srv::SetMode>::SharedFuture result) {
      client.on_waiting.store(false);
      if (result.get()->success) {
        client.mode.store(mode);
      }
    });
}

}  // namespace aurora::serial_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(aurora::serial_driver::SerialDriverNode)
