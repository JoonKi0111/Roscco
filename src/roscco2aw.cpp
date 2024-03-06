#include "rclcpp/rclcpp.hpp"
#include <memory>
#include "roscco_msgs/msg/can_info.hpp"
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"

#define KPH2MPS 1/3.6

class Roscco2AW : public rclcpp::Node
{
    public:
    Roscco2AW() : Node("roscco2aw")
    {
        sub_caninfo = this->create_subscription<roscco_msgs::msg::CANInfo>(
            "can_info", 10, std::bind(&Roscco2AW::topic_callback, this, std::placeholders::_1));
        pub_steer = this->create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>(
            "/vehicle/status/steering_status", 10);        
        pub_velocity = this->create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>(
            "/vehicle/status/velocity_status", 10);      
    }

    private:

    rclcpp::Subscription<roscco_msgs::msg::CANInfo>::SharedPtr sub_caninfo;
    rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr pub_steer;
    rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr pub_velocity;

    autoware_auto_vehicle_msgs::msg::SteeringReport steer_msg_;
    autoware_auto_vehicle_msgs::msg::VelocityReport velocity_msg_;

    void topic_callback(const roscco_msgs::msg::CANInfo &msg)
    {
        steer_msg_.stamp = msg.header.stamp;
        steer_msg_.steering_tire_angle = (msg.angle/15.7);
        pub_steer->publish(steer_msg_);

        velocity_msg_.header.stamp = msg.header.stamp;
        velocity_msg_.header.frame_id = "base_link";
        velocity_msg_.longitudinal_velocity = msg.speed * KPH2MPS;
        pub_velocity->publish(velocity_msg_);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<Roscco2AW> roscco2aw_obj = std::make_shared<Roscco2AW>();
    rclcpp::spin(roscco2aw_obj);
    rclcpp::shutdown();
    return 0;
}