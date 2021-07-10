
#include "rclcpp/rclcpp.hpp"
#include "detection_2d_lidar/detection_2d_lidar_node.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Detection2DLidarNode>());
  rclcpp::shutdown();
  return 0;
}
