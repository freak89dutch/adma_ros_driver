#include "adma_tools_cpp/gsda_server.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<genesys::tools::GSDAServer>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
