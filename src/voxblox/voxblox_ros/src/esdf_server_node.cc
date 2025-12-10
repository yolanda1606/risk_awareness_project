#include "voxblox_ros/esdf_server.h"

#include <gflags/gflags.h>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  // Create the ROS 2 node first
  auto node = std::make_shared<rclcpp::Node>("esdf_server");

  // Pass the node to the EsdfServer
  auto esdf_server = std::make_shared<voxblox::EsdfServer>(node);

  // Spin the node (not the server)
  rclcpp::spin(node);
  return 0;
}