#include "adma_tools_cpp/bag2gsdb_converter.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<genesys::tools::Bag2GSDBConverter>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}