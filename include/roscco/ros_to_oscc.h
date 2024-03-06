#ifndef ROS_TO_OSCC_H
#define ROS_TO_OSCC_H

#include <signal.h>

extern "C" {
#include <oscc.h>
}

#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <roscco_msgs/msg/brake_command.hpp>
#include <roscco_msgs/msg/enable_disable.hpp>
#include <roscco_msgs/msg/steering_command.hpp>
#include <roscco_msgs/msg/throttle_command.hpp>

class RosToOscc : public rclcpp::Node
{
public:
  /**
   * @brief RosToOscc class initializer
   *
   * This function constructs ROS subscribers which can publish messages to OSCC API.
   *

   */
  //  * @param public_nh  The public node handle to use for ROS subscribers.
  //  * @param private_nh The private node handle for ROS parameters.

  // RosToOscc(ros::NodeHandle* public_nh, ros::NodeHandle* private_nh);
  // explicit RosToOscc(const rclcpp::NodeOptions & node_options);

  explicit RosToOscc();
  /**
   * @brief Callback function to publish ROS BrakeCommand messages to OSCC.
   *
   * This function is a callback that consume a ROS BrakeCommand message and publishes them to the OSCC API.
   *
   * @param msg ROS BrakeCommand message to be consumed.
   */

  void brakeCommandCallback(const roscco_msgs::msg::BrakeCommand& msg);

  /**
   * @brief Callback function to publish ROS SteeringCommand messages to OSCC.
   *
   * This function is a callback that consumes a ROS SteeringCommand message and publishes them to the OSCC API.
   *
   * @param msg ROS SteeringCommand message to be consumed.
   */

  void steeringCommandCallback(const roscco_msgs::msg::SteeringCommand& msg);

  /**
   * @brief Callback function to publish ROS ThrottleCommand messages to OSCC.
   *
   * This function is a callback that consumes a ROS ThrottleCommand message and publishes them to the OSCC API.
   *
   * @param msg ROS ThrottleCommand message to be consumed.
   */
  void throttleCommandCallback(const roscco_msgs::msg::ThrottleCommand& msg);
  /**
   * @brief Callback function to publish ROS EnableDisable messages to OSCC.
   *
   * This function is a callback that consumes a ROS EnableDisable message and publishes them to the OSCC API.
   *
   * @param msg ROS EnableDisable message to be consumed.
   */
  void enableDisableCallback(const roscco_msgs::msg::EnableDisable& msg);
  //void canInfoCallback(const autoware_can_msgs::CANInfo::ConstPtr& msg); //AVC20_WS_200328

  //double steering_angle_report = 0.; //AVC20_WS_200328

  //void closedLoopControl( double setpoint, roscco::SteeringCommand& output, double steering_angle_report ); //AVC20_WS_200329

  //void canInfoCallback(const autoware_can_msgs::CANInfo::ConstPtr& msg); //AVC20_WS_200328

  //double steering_angle_report = 0.; //AVC20_WS_200328

  //void closedLoopControl( double setpoint, roscco::SteeringCommand& output, double steering_angle_report ); //AVC20_WS_200329

private:
  rclcpp::Subscription<roscco_msgs::msg::BrakeCommand>::SharedPtr topic_brake_command_;

  rclcpp::Subscription<roscco_msgs::msg::SteeringCommand>::SharedPtr topic_steering_command_;
  rclcpp::Subscription<roscco_msgs::msg::ThrottleCommand>::SharedPtr topic_throttle_command_;
  rclcpp::Subscription<roscco_msgs::msg::EnableDisable>::SharedPtr topic_enable_disable_command_;
};

//AVC20_WS_200328
/**
 * @brief setup a pid to control the steering angle
 * 
 * @param setpoint/target
 * @param command
 * @param steering angle position
 */
//void closedLoopControl( double setpoint, roscco::SteeringCommand& output, double steering_angle_report );

#endif  // ROS_TO_OSCC_H
