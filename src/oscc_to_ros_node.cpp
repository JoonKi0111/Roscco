#include <string>

extern "C" {
#include <oscc.h>
}

#include "rclcpp/rclcpp.hpp"
#include <memory>

#include <roscco/oscc_to_ros.h>
#include <roscco/ros_to_oscc.h>

void oscc(oscc_result_t ret_)
{
  if (ret_ != OSCC_OK)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"Could not initialize OSCC");
  }

  ret_ = oscc_disable();

  if (ret_ != OSCC_OK)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"Could not disable OSCC");
  }

  ret_ = oscc_close(0);

  if (ret_ != OSCC_OK)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"Could not close OSCC connection");

  }
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OsccToRos>();

  node->declare_parameter<int>("can_channel", 0);
  int can_channel = node->get_parameter<int>("can_channel",can_channel);
  
  oscc_result_t ret = OSCC_ERROR;

  ret = oscc_init();

  rclcpp::spin(node);

  oscc(ret);

  rclcpp::shutdown();
  return 0;
}

