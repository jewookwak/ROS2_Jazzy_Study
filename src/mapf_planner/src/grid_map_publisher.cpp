#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

class GridMapPublisher : public rclcpp::Node
{
public:
  GridMapPublisher()
  : Node("grid_map_publisher")
  {
    pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/grid_map", 10);

    // map 정보 초기화
    map_msg_.header.frame_id = "map";
    map_msg_.info.resolution = 0.1;
    map_msg_.info.width      = 12;
    map_msg_.info.height     = 22;
    map_msg_.info.origin.position.x = 0.0;
    map_msg_.info.origin.position.y = 0.0;
    map_msg_.info.origin.position.z = 0.0;
    map_msg_.info.origin.orientation.w = 1.0;
    map_msg_.data.assign(
      map_msg_.info.width * map_msg_.info.height,
      static_cast<int8_t>(0)
    );

    // 1 Hz로 publish
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&GridMapPublisher::on_timer, this)
    );
  }

private:
  void on_timer()
  {
    map_msg_.header.stamp = this->now();
    // 필요시 map_msg_.data[...] 업데이트
    pub_->publish(map_msg_);
  }

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  nav_msgs::msg::OccupancyGrid map_msg_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GridMapPublisher>());
  rclcpp::shutdown();
  return 0;
}
