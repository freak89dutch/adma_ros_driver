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
#include <adma_core_lib/parser/parser_utils.hpp>

#include "adma_ros2_delta_driver/adma_delta_driver.hpp"

namespace genesys
{
ADMADeltaDriver::ADMADeltaDriver(const rclcpp::NodeOptions & options)
: Node("adma_delta_driver", options)
{
  // define ROS parameters
  protocolVersion_ = this->declare_parameter("protocol_version", 1170);
  std::string jsonMappingFilePath = this->declare_parameter("addon_delta_mapping_path", "");
  // setup subscriber
  subDataRaw_ = this->create_subscription<adma_ros_driver_msgs::msg::Delta1170Raw>(
    "adma/addondelta_raw", 10, std::bind(
      &ADMADeltaDriver::rawDataCallback,
      this, std::placeholders::_1));
  // setup publisher
  pubDataScaled_ = this->create_publisher<adma_ros_driver_msgs::msg::Delta1170Scaled>(
    "adma/delta_scaled", 1);

  // setup UDP socket communication
  mapping_ = new genesys::parser::Mapping(protocolVersion_, jsonMappingFilePath);
}

ADMADeltaDriver::~ADMADeltaDriver() {}

void ADMADeltaDriver::rawDataCallback(adma_ros_driver_msgs::msg::Delta1170Raw::SharedPtr newMsg)
{
  std::array<char, 856> recv_buf_temp_;
  for (size_t i = 0; i < newMsg->data_size; i++) {
    recv_buf_temp_[i] = newMsg->raw_data[i];
  }
  adma_ros_driver_msgs::msg::Delta1170Scaled delta_msg_scaled;
  delta_msg_scaled.long_delta_distance = mapping_->loadDataFromBuffer<float, double>(
    "delta_scaled.long_delta_distance", recv_buf_temp_);
  delta_msg_scaled.long_delta_velocity = mapping_->loadDataFromBuffer<float, double>(
    "delta_scaled.long_delta_velocity", recv_buf_temp_);
  delta_msg_scaled.lat_delta_distance = mapping_->loadDataFromBuffer<float, double>(
    "delta_scaled.lat_delta_distance", recv_buf_temp_);
  delta_msg_scaled.lat_delta_velocity = mapping_->loadDataFromBuffer<float, double>(
    "delta_scaled.lat_delta_velocity", recv_buf_temp_);
  delta_msg_scaled.resultant_distance = mapping_->loadDataFromBuffer<float, double>(
    "delta_scaled.resultant_distance", recv_buf_temp_);
  delta_msg_scaled.resultant_velocity = mapping_->loadDataFromBuffer<float, double>(
    "delta_scaled.resultant_velocity", recv_buf_temp_);
  delta_msg_scaled.code_version = mapping_->loadDataFromBuffer<uint16_t, uint16_t>(
    "delta_scaled.code_version", recv_buf_temp_);
  delta_msg_scaled.angle_of_orientation = mapping_->loadDataFromBuffer<float, double>(
    "delta_scaled.angle_of_orientation", recv_buf_temp_);
  delta_msg_scaled.delta_time = mapping_->loadDataFromBuffer<int32_t, int32_t>(
    "delta_scaled.delta_time", recv_buf_temp_);
  delta_msg_scaled.target_status = mapping_->loadDataFromBuffer<uint16_t, uint16_t>(
    "delta_scaled.target_status", recv_buf_temp_);
  delta_msg_scaled.hunter_status = mapping_->loadDataFromBuffer<uint16_t, uint16_t>(
    "delta_scaled.hunter_status", recv_buf_temp_);
  // fill msg header for scaled msg
  delta_msg_scaled.header = newMsg->header;
  // fill scaled msg with content
  // delta_msg_scaled.abd_header = delta_msg_raw.abd_header;

  delta_msg_scaled.target_forward_velocity = mapping_->loadDataFromBuffer<int16_t, double>(
    "delta_scaled.target_forward_velocity", recv_buf_temp_);
  delta_msg_scaled.hunter_forward_velocity = mapping_->loadDataFromBuffer<int16_t, double>(
    "delta_scaled.hunter_forward_velocity", recv_buf_temp_);
  delta_msg_scaled.target_forward_acceleration = mapping_->loadDataFromBuffer<int16_t, double>(
    "delta_scaled.target_forward_acceleration", recv_buf_temp_);
  delta_msg_scaled.hunter_forward_acceleration = mapping_->loadDataFromBuffer<int16_t, double>(
    "delta_scaled.hunter_forward_acceleration", recv_buf_temp_);
  delta_msg_scaled.target_lateral_velocity = mapping_->loadDataFromBuffer<int16_t, double>(
    "delta_scaled.target_lateral_velocity", recv_buf_temp_);
  delta_msg_scaled.hunter_lateral_velocity = mapping_->loadDataFromBuffer<int16_t, double>(
    "delta_scaled.hunter_lateral_velocity", recv_buf_temp_);
  delta_msg_scaled.target_lateral_acceleration = mapping_->loadDataFromBuffer<int16_t, double>(
    "delta_scaled.target_lateral_acceleration", recv_buf_temp_);
  delta_msg_scaled.hunter_lateral_acceleration = mapping_->loadDataFromBuffer<int16_t, double>(
    "delta_scaled.hunter_lateral_acceleration", recv_buf_temp_);
  delta_msg_scaled.target_pitch_angle = mapping_->loadDataFromBuffer<int16_t, double>(
    "delta_scaled.target_pitch_angle", recv_buf_temp_);
  delta_msg_scaled.hunter_pitch_angle = mapping_->loadDataFromBuffer<int16_t, double>(
    "delta_scaled.hunter_pitch_angle", recv_buf_temp_);
  // // modify individual values where required (e.g. coordinates, LSB factor)
  delta_msg_scaled.target_longitude = convertCoordinates(
    mapping_->loadDataFromBuffer<double,
    double>("delta_scaled.target_longitude", recv_buf_temp_));
  delta_msg_scaled.target_latitude = convertCoordinates(
    mapping_->loadDataFromBuffer<double,
    double>("delta_scaled.target_latitude", recv_buf_temp_));

  pubDataScaled_->publish(delta_msg_scaled);
}

double ADMADeltaDriver::convertCoordinates(double rawValue)
{
  // convert raw double value to bytes
  unsigned char raw_bytes[8];
  std::memcpy(raw_bytes, &rawValue, sizeof(double));
  // extract first 4 bytes as long
  int64_t part_1 = (raw_bytes[3] << 24) | (raw_bytes[2] << 16) | (raw_bytes[1] << 8) | raw_bytes[0];

  // extract last 4 bytes as float for decimal values of coordinates
  float part_2;
  unsigned char float_bytes[4] = {raw_bytes[4], raw_bytes[5], raw_bytes[6], raw_bytes[7]};
  std::memcpy(&part_2, float_bytes, sizeof(part_2));

  double coordinate = (static_cast<double>(part_1) + part_2);
  return getScaledValue(coordinate, 0.001);
}

}  // namespace genesys

RCLCPP_COMPONENTS_REGISTER_NODE(genesys::ADMADeltaDriver)
