#include <roscco/ros_to_oscc.h>

using namespace std::chrono_literals;

namespace roscco_component
{

RosToOscc::RosToOscc(const rclcpp::NodeOptions & node_options) : Node("ros_to_oscc", node_options)
{
  sigset_t mask;
  sigset_t orig_mask;

  sigemptyset(&mask);
  sigemptyset(&orig_mask);
  sigaddset(&mask, SIGIO);

  // Temporary block of OSCC SIGIO while initializing ROS publication to prevent
  // signal conflicts
  if (sigprocmask(SIG_BLOCK, &mask, &orig_mask) < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"Failed to block SIGIO");
  }

  topic_brake_command_ =
    this->create_subscription<roscco_msgs::msg::BrakeCommand>(
      "brake_command", rclcpp::QoS(1), std::bind(&RosToOscc::brakeCommandCallback,this,std::placeholders::_1));

  topic_steering_command_ =
    this->create_subscription<roscco_msgs::msg::SteeringCommand>(
      "steering_command", rclcpp::QoS(1), std::bind(&RosToOscc::steeringCommandCallback, this,std::placeholders::_1));

  topic_throttle_command_ =
    this->create_subscription<roscco_msgs::msg::ThrottleCommand>(
      "throttle_command", rclcpp::QoS(1), std::bind(&RosToOscc::throttleCommandCallback, this,std::placeholders::_1));

  topic_enable_disable_command_ =
    this->create_subscription<roscco_msgs::msg::EnableDisable>(
      "enable_disable", rclcpp::QoS(1), std::bind(&RosToOscc::enableDisableCallback, this,std::placeholders::_1));

  topic_time_ =this->create_publisher<std_msgs::msg::Header>("time_from_roscco", rclcpp::QoS(1));
  pub_roscco_status_ =this->create_publisher<roscco_msgs::msg::RosccoStatus>("/roscco_status", rclcpp::QoS(1));

  timer_ = this->create_wall_timer(500ms, std::bind(&RosToOscc::timer_callback, this));

  if (sigprocmask(SIG_SETMASK, &orig_mask, NULL) < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"Failed to unblock SIGIO");
  }
}

void RosToOscc::brakeCommandCallback(const roscco_msgs::msg::BrakeCommand& msg)
{

  oscc_result_t ret = OSCC_ERROR;

  ret = oscc_publish_brake_position(msg.brake_position); 

  if (ret == OSCC_ERROR)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"OSCC_ERROR occured while trying send the brake position.");
  }
  else if (ret == OSCC_WARNING)
  {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"OSCC_WARNING occured while trying send the brake position.");
  }

}

void RosToOscc::steeringCommandCallback(const roscco_msgs::msg::SteeringCommand& msg)
{    
  oscc_result_t ret = OSCC_ERROR;
  
  ret = oscc_publish_steering_torque(msg.steering_torque);
  if (ret == OSCC_ERROR)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"OSCC_ERROR occured while trying send the steering torque.");
  }
  else if (ret == OSCC_WARNING)
  {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"OSCC_WARNING occured while trying send the brake position.");
  }
}

void RosToOscc::throttleCommandCallback(const roscco_msgs::msg::ThrottleCommand& msg)
{

  oscc_result_t ret = OSCC_ERROR;
  ret = oscc_publish_throttle_position(msg.throttle_position);

  if (ret == OSCC_ERROR)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"OSCC_ERROR occured while trying send the throttle position.");
  }
  else if (ret == OSCC_WARNING)
  {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"OSCC_WARNING occured while trying send the throttle position.");
  }
};

void RosToOscc::enableDisableCallback(const roscco_msgs::msg::EnableDisable& msg)
{
  Oscc_custom_result custom_result;

  custom_result = msg.enable_control ? oscc_enable() : oscc_disable();

  if (custom_result.result == OSCC_ERROR)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"OSCC_ERROR occured while trying to enable or disable control.");
  }
  else if (custom_result.result == OSCC_WARNING)
  {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"OSCC_WARNING occured while trying to enable or disable control.");
  }

  roscco_msgs::msg::RosccoStatus status_msg;

  if(msg.enable_control == true)
  {
    if(custom_result.steering_result == OSCC_OK) status_msg.steering_status = true;
    else status_msg.steering_status = false;
    if(custom_result.brake_result == OSCC_OK) status_msg.brake_status = true;
    else status_msg.brake_status = false;
    if(custom_result.throttle_result == OSCC_OK) status_msg.throttle_status = true;
    else status_msg.throttle_status = false;
  }
  else if(msg.enable_control == false)
  {
    if(custom_result.steering_result == OSCC_OK) status_msg.steering_status = false;
    else status_msg.steering_status = true;
    if(custom_result.brake_result == OSCC_OK) status_msg.brake_status = false;
    else status_msg.brake_status = true;
    if(custom_result.throttle_result == OSCC_OK) status_msg.throttle_status = false;
    else status_msg.throttle_status = true;
  }
  pub_roscco_status_->publish(status_msg);
}

void RosToOscc::timer_callback()
{
  std_msgs::msg::Header msg;
  msg.stamp = get_clock()->now();
  topic_time_ -> publish(msg);
}

} // namespace roscco_component

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(roscco_component::RosToOscc)
