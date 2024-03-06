// #include <roscco/oscc_to_ros.h>
#include <roscco/oscc_to_ros.h>
#include <iostream>
//AVC20_WS_200714
//#include <chrono>
//#include <thread>

// OsccToRos::OsccToRos(ros::NodeHandle* public_nh, ros::NodeHandle* private_nh)
// OsccToRos::OsccToRos(const rclcpp::NodeOptions & node_options) : Node("oscc_to_ros", node_options)
OsccToRos::OsccToRos() : Node("oscc_to_ros")
{
  sigset_t mask;
  sigset_t orig_mask;

  sigemptyset(&mask);
  sigemptyset(&orig_mask);
  sigaddset(&mask, SIGIO);

  // Temporary block of OSCC SIGIO while initializing ROS subscription to
  // prevent signal conflicts
  if (sigprocmask(SIG_BLOCK, &mask, &orig_mask) < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"Failed to block SIGIO");
  }

  topic_brake_report_= this->create_publisher<roscco_msgs::msg::BrakeReport>("brake_report", 10);

  topic_steering_report_ = this->create_publisher<roscco_msgs::msg::SteeringReport>("steering_report", 10);

  topic_throttle_report_ = this->create_publisher<roscco_msgs::msg::ThrottleReport>("throttle_report", 10);

  topic_fault_report_ = this->create_publisher<roscco_msgs::msg::FaultReport>("fault_report", 10);

  topic_obd_messages_ = this->create_publisher<roscco_msgs::msg::CanFrame>("can_frame", 10);

  topic_can_info_ = this->create_publisher<roscco_msgs::msg::CANInfo>("can_info", 10); //AVC20_WS_200326

  if (sigprocmask(SIG_SETMASK, &orig_mask, NULL) < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"Failed to unblock SIGIO");
  }

  oscc_subscribe_to_brake_reports(brake_callback);

  oscc_subscribe_to_steering_reports(steering_callback);

  oscc_subscribe_to_throttle_reports(throttle_callback);

  oscc_subscribe_to_fault_reports(fault_callback);

  oscc_subscribe_to_obd_messages(obd_callback);
};

void OsccToRos::steering_callback(oscc_steering_report_s* report)
{
  cast_callback<oscc_steering_report_s, roscco_msgs::msg::SteeringReport, roscco_msgs::msg::SteeringReportData>(report,
                                                                                            *topic_steering_report_);
}

void OsccToRos::brake_callback(oscc_brake_report_s* report)
{
  cast_callback<oscc_brake_report_s, roscco_msgs::msg::BrakeReport, roscco_msgs::msg::BrakeReportData>(report, *topic_brake_report_);
}

void OsccToRos::throttle_callback(oscc_throttle_report_s* report)
{

  cast_callback<oscc_throttle_report_s, roscco_msgs::msg::ThrottleReport, roscco_msgs::msg::ThrottleReportData>(report,
                                                                                             *topic_throttle_report_);
}

template <typename OSCCTYPE, typename ROSMSGTYPE, typename ROSDATATYPE>
void OsccToRos::cast_callback(OSCCTYPE* report, rclcpp::Publisher<ROSMSGTYPE>& pub)
{
  ROSMSGTYPE* ros_message(new ROSMSGTYPE);

  ROSDATATYPE* data = (ROSDATATYPE*)report;

  ros_message->data = *data;

  ros_message->header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();

  pub.publish(*ros_message);

  delete ros_message;
}

void OsccToRos::fault_callback(oscc_fault_report_s* report)
{
  roscco_msgs::msg::FaultReport* ros_message(new roscco_msgs::msg::FaultReport);

  // ROS does not pack the structs so individual assignment is required over
  // cast
  ros_message->data.magic[0] = report->magic[0];
  ros_message->data.magic[1] = report->magic[1];
  ros_message->data.fault_origin_id = report->fault_origin_id;
  ros_message->data.dtcs = report->dtcs;
  ros_message->data.reserved = report->reserved;

  // ros_message->header.stamp = ros::Time::now();
  ros_message->header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();

  topic_fault_report_->publish(*ros_message);

  delete ros_message;
}

void OsccToRos::obd_callback(can_frame* frame)
{
  //AVC20_WS_200325
  static double steering_angle_report = 0;
  static double speed_report = 0;

  static int test_var = 0;
  static int8_t direction = 1; 

  roscco_msgs::msg::CANInfo* ros_message(new roscco_msgs::msg::CANInfo);

  switch(frame->can_id)
  {
    case KIA_SOUL_OBD_STEERING_WHEEL_ANGLE_CAN_ID: //0x2B0 = 688
    {
      steering_angle_report = frame->data[0] + frame->data[1] * 256;
      //printf("frame->data[0] : %d \n", frame->data[0]);
      //printf("frame->data[1] : %d \n", frame->data[1]);
      if(steering_angle_report > 60000) 
      {
        steering_angle_report -= 65535;
      }
      
      steering_angle_report *= DEG2RAD;
      steering_angle_report /= 10;
      steering_angle_report += 0.06;

      ros_message->header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
      ros_message->angle = steering_angle_report;
      ros_message->speed = speed_report;
      topic_can_info_->publish(*ros_message);
      break; 
    }

    case KIA_SOUL_OBD_WHEEL_SPEED_CAN_ID: //0x4B0 = 1200 //AVC20_WS_200131
    {
      speed_report = 0.5 * ((frame->data[4] + frame->data[5] * 256) + (frame->data[6] + frame->data[7] * 256)); //rear //AVC20_WS_200416
      speed_report *= WHEEL_SPEED_RATIO;

      speed_report *= direction;

      break;
    }
    
    case TEST: //0x291
    {
      test_var = frame->data[2] + frame->data[3] * 256;

      if(test_var > 60000) 
      {
        test_var -= 65535;
      }

      if(test_var < 0)
      {
        direction = -1;
      }
      else if(test_var > 0)
      {
        direction = 1;
      }
      else
      {
        direction = direction;
      }
    }

    default:
    {
      break;
    }
  }
  delete ros_message;
  
}
