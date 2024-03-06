#include <roscco/ros_to_oscc.h>

// RosToOscc::RosToOscc(ros::NodeHandle* public_nh, ros::NodeHandle* private_nh)
// RosToOscc::RosToOscc(const rclcpp::NodeOptions & node_options) 
// : Node("ros_to_oscc", node_options)
RosToOscc::RosToOscc() : Node("ros_to_oscc")
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
      this->create_subscription<roscco_msgs::msg::BrakeCommand>("brake_command", 10, std::bind(&RosToOscc::brakeCommandCallback,this,std::placeholders::_1));

  topic_steering_command_ =
      this->create_subscription<roscco_msgs::msg::SteeringCommand>("steering_command", 10, std::bind(&RosToOscc::steeringCommandCallback, this,std::placeholders::_1));

  topic_throttle_command_ =
      this->create_subscription<roscco_msgs::msg::ThrottleCommand>("throttle_command", 10, std::bind(&RosToOscc::throttleCommandCallback, this,std::placeholders::_1));

  topic_enable_disable_command_ =
     this->create_subscription<roscco_msgs::msg::EnableDisable>("enable_disable", 10, std::bind(&RosToOscc::enableDisableCallback, this,std::placeholders::_1));

  //topic_can_info_ = public_nh->subscribe<autoware_can_msgs::CANInfo>("can_info", 10, &RosToOscc::canInfoCallback, this); //AVC20_WS_200328

  //topic_pid_report_ = public_nh->advertise<roscco::PIDReport>("pid_report", 10); //AVC20_WS_200329 

  if (sigprocmask(SIG_SETMASK, &orig_mask, NULL) < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"Failed to unblock SIGIO");
  }
};

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
};

void RosToOscc::steeringCommandCallback(const roscco_msgs::msg::SteeringCommand& msg)
{
  oscc_result_t ret = OSCC_ERROR;
  
  //AVC20_WS_200328
  //roscco::SteeringCommand output;
  //output.header.stamp = ros::Time::now();
  //closedLoopControl(msg->steering_torque, output, steering_angle_report); //msg->steering_torque : angle_command, steering_angle_report : angle_measurement, output : torque_command
  //ret = oscc_publish_steering_torque(output.steering_torque); //segmentation fault
  ret = oscc_publish_steering_torque(msg.steering_torque);
  if (ret == OSCC_ERROR)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"OSCC_ERROR occured while trying send the steering torque.");
  }
  else if (ret == OSCC_WARNING)
  {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"OSCC_WARNING occured while trying send the steering torque.");
  }
};

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
  oscc_result_t ret = OSCC_ERROR;

  ret = msg.enable_control ? oscc_enable() : oscc_disable();

  if (ret == OSCC_ERROR)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"OSCC_ERROR occured while trying to enable or disable control.");
  }
  else if (ret == OSCC_WARNING)
  {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"OSCC_WARNING occured while trying to enable or disable control.");
  }
}
