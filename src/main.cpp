/*
 * Translates sensor_msgs/NavSat{Fix,Status} into nav_msgs/Odometry using UTM
 */

#include <memory>

// #include "odom_from_navrelposned/agrorob_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "odom_from_navrelposned/dgps.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "odom_from_navrelposned started");
  rclcpp::spin(std::make_shared<odom_from_navrelposned::DGPS>());
  rclcpp::shutdown();
  return 0;
}
