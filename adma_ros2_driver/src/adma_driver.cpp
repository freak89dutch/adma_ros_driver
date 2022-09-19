#include "adma_ros2_driver/adma_driver.hpp"
#include "adma_ros2_driver/adma_parse.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace genesys
{
        ADMADriver::ADMADriver(const rclcpp::NodeOptions &options) : Node("adma_driver", options)
        {
                std::string param_address = this->declare_parameter("destination_ip", "0.0.0.0");
                _address = boost::asio::ip::address::from_string(param_address);
                _port = this->declare_parameter("destination_port", 1040);
                _performance_check = this->declare_parameter("use_performance_check", false);
                _gnss_frame = this->declare_parameter("gnss_frame", "gnss_link");
                _imu_frame = this->declare_parameter("imu_frame", "imu_link");

                _pub_adma_data = this->create_publisher<adma_msgs::msg::AdmaData>("adma/data", 1);
                _pub_navsat_fix = this->create_publisher<sensor_msgs::msg::NavSatFix>("adma/fix", 1);
                _pub_imu = this->create_publisher<sensor_msgs::msg::Imu>("adma/imu", 1);
                _pub_heading = this->create_publisher<std_msgs::msg::Float64>("adma/heading", 1);
                _pub_velocity = this->create_publisher<std_msgs::msg::Float64>("adma/velocity", 1);

                updateLoop();
        }

        ADMADriver::~ADMADriver(){

        }

        void ADMADriver::updateLoop()
        {
                /* Create an IO Service wit the OS given the IP and the port */
                boost::asio::io_service io_service;
                /* Establish UDP connection*/
                boost::asio::ip::udp::endpoint local_endpoint = boost::asio::ip::udp::endpoint(_address, _port);
                // RCLCPP_INFO(get_logger(), "binding to: %s:%d", _address.to_string(), _port);

                /* Socket handling */
                boost::asio::ip::udp::socket socket(io_service);
                socket.open(boost::asio::ip::udp::v4());
                socket.bind(local_endpoint);
                boost::asio::ip::udp::endpoint sender_endpoint;
                while (rclcpp::ok())
                {
                        /* The length of the stream from ADMA is 856 bytes */
                        std::array<char, 856> recv_buf;
                        _len = socket.receive_from(boost::asio::buffer(recv_buf), sender_endpoint);

                        RCLCPP_INFO(get_logger(), "received payload: %ld", _len);
                        /* Prepare for parsing */
                        std::string local_data(recv_buf.begin(), recv_buf.end());
                        /* Load the messages on the publishers */
                        // adma_msgs::msg::AdmaData message;
                        // sensor_msgs::msg::NavSatFix message_fix;
                        // message_fix.header.stamp = this->now();
                        // message_fix.header.frame_id = "adma";
                        // std_msgs::msg::Float64 message_heading;
                        // std_msgs::msg::Float64 message_velocity;
                        // message.timemsec = this->get_clock()->now().seconds() * 1000;
                        // message.timensec = this->get_clock()->now().nanoseconds();
                        // getparseddata(local_data, message, message_fix, message_heading, message_velocity);
                        
                        // /* publish the ADMA message */
                        // _pub_adma_data->publish(message);
                        // _pub_navsat_fix->publish(message_fix);
                        // _pub_heading->publish(message_heading);
                        // _pub_velocity->publish(message_velocity);
                        // double grab_time = this->get_clock()->now().seconds();

                        // if (_performance_check)
                        // {
                        //         char ins_time_msec[] = {local_data[584], local_data[585], local_data[586], local_data[587]};
                        //         memcpy(&message.instimemsec, &ins_time_msec, sizeof(message.instimemsec));
                        //         float weektime = message.instimeweek;
                        //         RCLCPP_INFO(get_logger(), "%f ", ((grab_time * 1000) - (message.instimemsec + 1592697600000)));
                        // }

                        adma_msgs::msg::AdmaData message_admadata;
                        message_admadata.timemsec = this->get_clock()->now().seconds() * 1000;
                        message_admadata.timensec = this->get_clock()->now().nanoseconds();
                        sensor_msgs::msg::NavSatFix message_navsatfix;
                        message_navsatfix.header.stamp = this->get_clock()->now();
                        message_navsatfix.header.frame_id = _gnss_frame;
                        sensor_msgs::msg::Imu message_imu;
                        message_imu.header.stamp = this->get_clock()->now();
                        message_imu.header.frame_id = _imu_frame;

                        std_msgs::msg::Float64 message_heading;
                        std_msgs::msg::Float64 message_velocity;
                        getparseddata(local_data, message_admadata, message_navsatfix, message_imu, message_heading, message_velocity);

                        /* publish the ADMA message */
                        _pub_adma_data->publish(message_admadata);
                        _pub_navsat_fix->publish(message_navsatfix);
                        _pub_imu->publish(message_imu);
                        _pub_heading->publish(message_heading);
                        _pub_velocity->publish(message_velocity);

                        if (_performance_check)
                        {
                        double grab_time = this->get_clock()->now().seconds();
                        char ins_time_msec[] = { local_data[584], local_data[585], local_data[586], local_data[587] };
                        memcpy(&message_admadata.instimemsec, &ins_time_msec, sizeof(message_admadata.instimemsec));
                        float weektime = message_admadata.instimeweek;
                        RCLCPP_INFO(this->get_logger(), "%f ", ((grab_time * 1000) - (message_admadata.instimemsec + 1592697600000)));
                        }
                }
        }
} // namespace genesys

RCLCPP_COMPONENTS_REGISTER_NODE(genesys::ADMADriver)