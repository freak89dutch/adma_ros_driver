#pragma once

#include <bitset>

#include "adma_ros2_driver/data/adma_data_v334.hpp"
#include "adma_ros_driver_msgs/msg/adma_data_scaled.hpp"
#include "adma_ros_driver_msgs/msg/adma_status.hpp"
#include "adma_ros_driver_msgs/msg/byte_error_warning.hpp"
#include "adma_ros_driver_msgs/msg/byte_status.hpp"
#include "adma_ros_driver_msgs/msg/error_warning.hpp"
#include "adma_ros_driver_msgs/msg/status.hpp"

class ADMA2ROSParserV334
{
public:
  ADMA2ROSParserV334();
  ~ADMA2ROSParserV334() {}
  void mapAdmaMessageToROS(
    adma_ros_driver_msgs::msg::AdmaDataScaled & ros_msg, AdmaDataV334 & adma_data);
  void mapStatusToROS(adma_ros_driver_msgs::msg::AdmaStatus & ros_msg, AdmaDataV334 & adma_data);
  void mapAdmaHeader(adma_ros_driver_msgs::msg::AdmaDataScaled & ros_msg, AdmaDataV334 & adma_data);
  void mapStatusBytes(
    adma_ros_driver_msgs::msg::ByteStatus & ros_msg_byte_status, AdmaDataV334 & adma_data);
  void mapStatusBitfields(
    adma_ros_driver_msgs::msg::Status & ros_msg_status, AdmaDataV334 & adma_data);
  void mapErrorWarningBytes(
    adma_ros_driver_msgs::msg::ByteErrorWarning & ros_msg_byte_error_warning,
    AdmaDataV334 & adma_data);
  void mapErrorWarningBitfields(
    adma_ros_driver_msgs::msg::ErrorWarning & ros_msg_error_warning, AdmaDataV334 & adma_data);
  void mapUnscaledData(
    adma_ros_driver_msgs::msg::AdmaDataScaled & ros_msg, AdmaDataV334 & adma_data);
  void mapScaledData(adma_ros_driver_msgs::msg::AdmaDataScaled & ros_msg, AdmaDataV334 & adma_data);
  void mapPOI(adma_ros_driver_msgs::msg::AdmaDataScaled & ros_msg, AdmaDataV334 & adma_data);
};
