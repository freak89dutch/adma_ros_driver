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

#include "adma_ros2_driver/adma_driver.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <adma_core_lib/parser/parser_utils.hpp>
#include <adma_core_lib/parser/mapping.hpp>
#include <adma_ros_driver_msgs/msg/admanet_header.hpp>

namespace genesys
{
ADMADriver::ADMADriver(const rclcpp::NodeOptions & options)
: Node("adma_driver", options),
  qosProfile_(rclcpp::KeepLast(10))
{
  // define ROS parameters, adjustable by config yaml file
  performanceCheck_ = this->declare_parameter("use_performance_check", false);
  jsonMappingFilePath_ = this->declare_parameter("admanet_mapping_path", "");

  gnss_frame_ = this->declare_parameter("frame_ids.navsatfix", "gnss_link");
  imu_frame_ = this->declare_parameter("frame_ids.imu", "imu_link");
  adma_frame_ = this->declare_parameter("frame_ids.adma", "adma");
  adma_status_frame_ = this->declare_parameter("frame_ids.adma_status", "adma_status");
  raw_data_frame_ = this->declare_parameter("frame_ids.raw_data", "data_raw");
  odometry_pose_frame_ = this->declare_parameter("frame_ids.odometry_pose_id", "adma");
  odometry_child_frame_ = this->declare_parameter("frame_ids.odometry_twist_id", "odometry");

  odometry_yaw_offset_ = this->declare_parameter("odometry_yaw_offset", 0.0);
  navsatfix_id_ = this->declare_parameter("topic_pois.navsatfix", 1);
  imu_id_ = this->declare_parameter("topic_pois.imu", 1);
  velocity_id_ = this->declare_parameter("topic_pois.velocity", 1);
  odometry_id_ = this->declare_parameter("topic_pois.odometry", 1);
  time_mode_ = this->declare_parameter("time_mode", 0);  // 0 / 1
  publish_clock_ = this->declare_parameter("publish_clock", false);

  // setup publisher that are protocol version indepent
  qosProfile_.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  pub_navsat_fix_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("adma/fix", qosProfile_);
  pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("adma/imu", qosProfile_);
  pub_heading_ = this->create_publisher<std_msgs::msg::Float64>("adma/heading", qosProfile_);
  pub_velocity_ = this->create_publisher<std_msgs::msg::Float64>("adma/velocity", qosProfile_);

  subRawData_ = create_subscription<adma_ros_driver_msgs::msg::AdmaDataRaw>(
    "adma/admanet_raw", 10, std::bind(
      &ADMADriver::rawDataCallback,
      this, std::placeholders::_1));

  if (publish_clock_) {
    pub_clock_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", qosProfile_);
  }

  oldTimestampForMsgs_ = this->get_clock()->now();
}

ADMADriver::~ADMADriver() {}

void ADMADriver::rawDataCallback(adma_ros_driver_msgs::msg::AdmaDataRaw::SharedPtr newMsg)
{
  // first convert the received raw ROS msg into byte array for easier parsing afterwards
  std::array<char, 856> recv_buf;
  for (size_t i = 0; i < newMsg->size; i++) {
    recv_buf[i] = newMsg->raw_data[i];
  }

  builtin_interfaces::msg::Time startTime = this->get_clock()->now();
  // prepare several ros msgs
  sensor_msgs::msg::NavSatFix message_fix;
  message_fix.header.frame_id = imu_frame_;
  std_msgs::msg::Float64 message_heading;
  std_msgs::msg::Float64 message_velocity;
  sensor_msgs::msg::Imu message_imu;
  message_imu.header.frame_id = imu_frame_;

  builtin_interfaces::msg::Time timestampForMsgs;
  // offset between UNIX and GNSS (in ms)
  uint64_t offset_gps_unix = 315964800000;
  uint64_t week_to_msec = 604800000;
  uint64_t timestamp;

  // first extract admanet header to get the protocol version
  adma_ros_driver_msgs::msg::AdmanetHeader admaHeaderMsg;
  extractAdmanetHeader(admaHeaderMsg, recv_buf);


  if (!setupDone) {
    RCLCPP_INFO(get_logger(), "Receiving Admanet version: %d", admaHeaderMsg.format_version);
    // prevent invalid setup caused by broken UDP packets
    if (admaHeaderMsg.format_version < 3200 || admaHeaderMsg.format_version > 10000) {
      RCLCPP_INFO(get_logger(), "Admanet version invalid, waiting for valid data..");
      return;
    }
    if (admaHeaderMsg.format_version == 3200) {
      // for version 3.2 we use the old message format
      pub_adma_data_ = this->create_publisher<adma_ros_driver_msgs::msg::AdmaData>(
        "adma/data",
        qosProfile_);
    } else {
      // setup publisher for all newer versions (>= 3.3.3)
      pub_adma_data_scaled_ =
        this->create_publisher<adma_ros_driver_msgs::msg::AdmaDataScaled>(
        "adma/data_scaled",
        qosProfile_);
      pub_adma_status_ =
        this->create_publisher<adma_ros_driver_msgs::msg::AdmaStatus>("adma/status", qosProfile_);
      pub_odometry_ =
        this->create_publisher<nav_msgs::msg::Odometry>("adma/odometry", qosProfile_);
    }
    // setup parser and finish setup
    parser_ = new ADMA2ROSParser(admaHeaderMsg.format_version, jsonMappingFilePath_);
    setupDone = true;
  }

  // read Adma msg from UDP data packet
  if (admaHeaderMsg.format_version == 3200) {
    adma_ros_driver_msgs::msg::AdmaData admaData_ros_msg;
    admaData_ros_msg.adma_header = admaHeaderMsg;
    parser_->mapAdmaMessageToROS(admaData_ros_msg, recv_buf);
    timestamp = admaData_ros_msg.instimemsec + offset_gps_unix;
    timestamp += admaData_ros_msg.instimeweek * week_to_msec;
    admaData_ros_msg.timemsec = timestamp;
    admaData_ros_msg.timensec = timestamp * 1E6;

    // read NavSatFix out of AdmaData
    parser_->extractNavSatFix(admaData_ros_msg, message_fix);

    // read heading and velocity
    message_heading.data = admaData_ros_msg.finsyaw;
    message_velocity.data =
      std::sqrt(
      std::pow(admaData_ros_msg.fgpsvelframex, 2) + std::pow(admaData_ros_msg.fgpsvelframey, 2)) *
      3.6;

    // read IMU
    parser_->extractIMU(admaData_ros_msg, message_imu);
    admaData_ros_msg.header.stamp.sec = timestamp / 1000;
    admaData_ros_msg.header.stamp.nanosec = (timestamp % 1000) * 1E6;
    pub_adma_data_->publish(admaData_ros_msg);

  } else {
    adma_ros_driver_msgs::msg::AdmaDataScaled adma_data_scaled_msg;
    adma_data_scaled_msg.adma_header = admaHeaderMsg;
    adma_ros_driver_msgs::msg::AdmaStatus status_msg;
    parser_->extractAdmaDataScaled(adma_data_scaled_msg, recv_buf);
    parser_->extractPOIs(adma_data_scaled_msg, recv_buf);
    parser_->extractHeading(message_heading, recv_buf);
    parser_->extractAdmaStatus(status_msg, recv_buf);
    adma_data_scaled_msg.status = status_msg.status;

    adma_data_scaled_msg.header.frame_id = adma_frame_;
    // define POI-list for publishing odometry
    pois = {
      adma_data_scaled_msg.poi_1,
      adma_data_scaled_msg.poi_2,
      adma_data_scaled_msg.poi_3,
      adma_data_scaled_msg.poi_4,
      adma_data_scaled_msg.poi_5,
      adma_data_scaled_msg.poi_6,
      adma_data_scaled_msg.poi_7,
      adma_data_scaled_msg.poi_8
    };
    timestamp = adma_data_scaled_msg.ins_time_msec + offset_gps_unix;
    timestamp += adma_data_scaled_msg.ins_time_week * week_to_msec;
    adma_data_scaled_msg.time_msec = timestamp;
    adma_data_scaled_msg.time_nsec = (timestamp % 1000) * 1E6;

    if (time_mode_ == 0) {
      // mode == 0 -> use ADMA time
      timestampForMsgs.sec = timestamp / 1000;
      timestampForMsgs.nanosec = (timestamp % 1000) * 1E6;
    } else if (time_mode_ == 1) {
      // mode == 1 -> use current ROS system time
      timestampForMsgs = get_clock()->now();
    }

    adma_data_scaled_msg.header.stamp = timestampForMsgs;

    parser_->extractNavSatFix(adma_data_scaled_msg, message_fix, pois, navsatfix_id_);
    parser_->extractIMU(adma_data_scaled_msg, message_imu, pois, imu_id_);

    // fill odometry message
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.frame_id = odometry_pose_frame_;
    odom_msg.child_frame_id = odometry_child_frame_;
    odom_msg.header.stamp = timestampForMsgs;
    parser_->extractOdometry(
      adma_data_scaled_msg, odom_msg, odometry_yaw_offset_, pois,
      odometry_id_);


    // read heading and velocity
    message_heading.data = adma_data_scaled_msg.ins_yaw;
    geometry_msgs::msg::Vector3 insSource = velocity_id_ == 0 ?
      adma_data_scaled_msg.ins_vel_frame :
      pois[velocity_id_ - 1].ins_vel_hor;
    message_velocity.data = std::sqrt(std::pow(insSource.x, 2) + std::pow(insSource.y, 2)) * 3.6;

    status_msg.header.stamp = timestampForMsgs;
    status_msg.header.frame_id = adma_status_frame_;

    // kind of a "hack" to ensure clock is only published if INS time is valid
    if (adma_data_scaled_msg.ins_time_week > 0 && publish_clock_) {
      rosgraph_msgs::msg::Clock clockMsg;
      clockMsg.clock = timestampForMsgs;
      pub_clock_->publish(clockMsg);
    }

    // publish the ADMAnet version specific messages
    pub_odometry_->publish(odom_msg);
    pub_adma_data_scaled_->publish(adma_data_scaled_msg);
    pub_adma_status_->publish(status_msg);
  }


  // publish the ADMAnet version indepent messages
  message_fix.header.stamp = timestampForMsgs;
  message_imu.header.stamp = timestampForMsgs;
  pub_navsat_fix_->publish(message_fix);
  pub_heading_->publish(message_heading);
  pub_velocity_->publish(message_velocity);
  pub_imu_->publish(message_imu);

  // just for debugging
  if (performanceCheck_) {
    builtin_interfaces::msg::Time endTime = this->get_clock()->now();
    rclcpp::Duration duration = rclcpp::Time(endTime) - rclcpp::Time(startTime);
    RCLCPP_INFO(get_logger(), " parsing time: %f ", duration.seconds());
  }

  rclcpp::Duration frequency = rclcpp::Time(timestampForMsgs) - rclcpp::Time(oldTimestampForMsgs_);
  // RCLCPP_INFO(
  //   get_logger(), "time between msgs: %f  / Frequency: %f", frequency.seconds(),
  //   (1 / frequency.seconds()));
  // RCLCPP_INFO(
  //   get_logger(), "Timestamp INS: %d / %d", timestampForMsgs.sec,
  //   timestampForMsgs.nanosec);
  // RCLCPP_INFO(
  //   get_logger(), "Timestamp old: %d / %d", oldTimestampForMsgs_.sec, oldTimestampForMsgs_.nanosec);
  oldTimestampForMsgs_ = timestampForMsgs;
}

}  // namespace genesys

RCLCPP_COMPONENTS_REGISTER_NODE(genesys::ADMADriver)
