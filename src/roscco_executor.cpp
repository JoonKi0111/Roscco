#include <memory>
#include "roscco/ros_to_oscc.h"
#include "rclcpp/rclcpp.hpp"

void oscc(oscc_result_t ret_, const int &can_channel)
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

  ret_ = oscc_close(can_channel);

  if (ret_ != OSCC_OK)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"Could not close OSCC connection");
  }
}

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  oscc_result_t ret = OSCC_ERROR;
  ret = oscc_init();
  auto RosToOscc = std::make_shared<roscco_component::RosToOscc>(options);
  exec.add_node(RosToOscc);
  exec.spin();
  oscc(ret, 1);
  rclcpp::shutdown();
  return 0;
}
