// BSD 3-Clause License
// Copyright (c) 2023, GeneSys Elektronik
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <rclcpp_components/register_node_macro.hpp>
#include <adma_ros_driver_msgs/msg/admanet_header.hpp>
#include "adma_core_lib/parser/parser_utils.hpp"
#include "adma_core_lib/adma_server.hpp"

namespace genesys
{
ADMAServer::ADMAServer(const rclcpp::NodeOptions & options)
: Node("adma_server", options)
{
  // declare ROS parameters
  std::string admaAddress = this->declare_parameter("adma_ip", "0.0.0.0");
  int admaNetPort = this->declare_parameter("admanet_port", 1025);
  int addonDeltaPort = this->declare_parameter("addondelta_port", 1026);
  timeMode_ = this->declare_parameter("time_mode", 0);

  // setup ADMANet socket
  admaNetLen_ = 856;
  admaNetSocket_ = new genesys::core::UDPSocket(admaNetLen_);
  admaNetSocket_->setupReceiveSocket(admaAddress, admaNetPort);
  // TODO(rschilli): make AddonDelta optional (or disconnect after timeout)
  // setup AddonDelta socket
  addonDeltaLen_ = 88;
  addonDeltaSocket_ = new genesys::core::UDPSocket(addonDeltaLen_);
  addonDeltaSocket_->setupReceiveSocket(admaAddress, addonDeltaPort);

  pubAdmaNetRaw_ =
    this->create_publisher<adma_ros_driver_msgs::msg::AdmaDataRaw>(
    "adma/admanet_raw", 1);
  pubAddonDeltaRaw_ =
    this->create_publisher<adma_ros_driver_msgs::msg::Delta1170Raw>(
    "adma/addondelta_raw", 1);

  updateLoop();

}

ADMAServer::~ADMAServer()
{
  admaNetSocket_->~UDPSocket();
  addonDeltaSocket_->~UDPSocket();
}

void ADMAServer::updateLoop()
{
  std::array<char, 856> admanetRecvBuf;
  std::array<char, 88> addonDeltaRecvBuf;

  builtin_interfaces::msg::Time timestampForMsgs;
  // offset between UNIX and GNSS (in ms)
  uint64_t offset_gps_unix = 315964800000;
  uint64_t week_to_msec = 604800000;
  uint64_t timestamp;

  while (rclcpp::ok()) {
    admaNetSocket_->receiveUDPPacket(admanetRecvBuf);
    adma_ros_driver_msgs::msg::AdmaDataRaw admanetRawMsg;

    // first extract admanet header
    adma_ros_driver_msgs::msg::AdmanetHeader admaHeaderMsg;
    extractAdmanetHeader(admaHeaderMsg, admanetRecvBuf);
    if (timeMode_ == 0) {
      // mode == 0 -> use ADMA time
      uint32_t insTimeMsec;
      uint16_t insTimeWeek;
      extractINSTime(admanetRecvBuf, insTimeMsec, insTimeWeek);
      timestamp = insTimeMsec + offset_gps_unix;
      timestamp += insTimeWeek * week_to_msec;
      timestampForMsgs.sec = timestamp / 1000;
      timestampForMsgs.nanosec = (timestamp % 1000) * 1E6;
    } else if (timeMode_ == 1) {
      // mode == 1 -> use current ROS system time
      timestampForMsgs = get_clock()->now();
    }
    // fill ADMAnet msg
    admanetRawMsg.size = admaNetLen_;
    admanetRawMsg.header.stamp = timestampForMsgs;
    admanetRawMsg.header.frame_id = "admanet_raw";
    for (size_t i = 0; i < admaNetLen_; ++i) {
      admanetRawMsg.raw_data.push_back(admanetRecvBuf[i]);
    }
    // publish ADMAnet data
    pubAdmaNetRaw_->publish(admanetRawMsg);

    // receive AddonDelta packets
    addonDeltaSocket_->receiveUDPPacket(addonDeltaRecvBuf);
    adma_ros_driver_msgs::msg::Delta1170Raw addondeltaRawMsg;
    // publish raw data as byte array
    addondeltaRawMsg.data_size = addonDeltaLen_;
    addondeltaRawMsg.header.stamp = timestampForMsgs;
    addondeltaRawMsg.header.frame_id = "addondelta_raw";
    // copy raw data
    for (size_t i = 0; i < addonDeltaLen_; ++i) {
      addondeltaRawMsg.raw_data.push_back(addonDeltaRecvBuf[i]);
    }
    pubAddonDeltaRaw_->publish(addondeltaRawMsg);

  }
}
}  // namespace genesys

RCLCPP_COMPONENTS_REGISTER_NODE(genesys::ADMAServer)
