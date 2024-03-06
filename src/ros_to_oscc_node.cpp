#include <string>

extern "C" {
    #include <oscc.h>
    }
    
    #include "rclcpp/rclcpp.hpp"
    #include <memory>
    
    #include <roscco/oscc_to_ros.h>
    #include <roscco/ros_to_oscc.h>

    int main(int argc, char* argv[])
    {
      rclcpp::init(argc, argv);
      auto node = std::make_shared<RosToOscc>();
    
      rclcpp::spin(node);
        
      rclcpp::shutdown();
      return 0;
    }
    
    