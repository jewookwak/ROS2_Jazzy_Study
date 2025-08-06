#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

class TrackedPathPublisher : public rclcpp::Node
{
public:
    TrackedPathPublisher() : Node("tracked_path_publisher")
    {
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("tracked_path", 10);
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/tracked_pose", 10,
            std::bind(&TrackedPathPublisher::pose_callback, this, std::placeholders::_1));

        // initialize header frame
        path_.header.frame_id = "map";
    }

private:
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        path_.header.stamp = this->now();
        path_.poses.push_back(*msg);  // 누적
        path_pub_->publish(path_);
    }

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    nav_msgs::msg::Path path_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrackedPathPublisher>());
    rclcpp::shutdown();
    return 0;
}
