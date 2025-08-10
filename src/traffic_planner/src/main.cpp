#include "rclcpp/rclcpp.hpp"

#ifdef USE_NODE02
#include "traffic_planner/TrafficPlannerNode02.hpp"
#else
#include "traffic_planner/TrafficPlannerNode.hpp"
#endif

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
#ifdef USE_NODE02
    RCLCPP_INFO(rclcpp::get_logger("main"), "🚀 Traffic Path Planner 시작 (20cm 해상도)");
#else
    RCLCPP_INFO(rclcpp::get_logger("main"), "🚀 Traffic Path Planner 시작 (10cm 해상도)");
#endif
    
    auto node = std::make_shared<TrafficPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}