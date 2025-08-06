#include "traffic_planner/TrafficPlannerNode.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv) {
    // ROS2 초기화
    rclcpp::init(argc, argv);
    
    try {
        // TrafficPlannerNode 생성 및 실행
        auto node = std::make_shared<TrafficPlannerNode>();
        
        // 노드 스핀
        rclcpp::spin(node);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "예외 발생: %s", e.what());
        rclcpp::shutdown();
        return -1;
    }
    
    // 정상 종료
    rclcpp::shutdown();
    return 0;
}