#include <array>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

class MovingObstaclesNode : public rclcpp::Node {
  public:
    MovingObstaclesNode() : Node("moving_obstacles")
    {
      speeds_[0] = this->declare_parameter<double>("top_speed", 0.8);
      speeds_[1] = this->declare_parameter<double>("bottom_speed", 0.6);
      for (std::size_t i = 0; i < names_.size(); ++i) {
        publishers_[i] = create_publisher<geometry_msgs::msg::Twist>("/" + names_[i] + "/cmd_vel", 1);
      }
      tf_pub_ = create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);
      poses_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
          "/simulation_poses", rclcpp::QoS(1).best_effort(),
          [this](tf2_msgs::msg::TFMessage::ConstSharedPtr msg) {
            for (const auto& transform : msg->transforms) {
              for (std::size_t i = 0; i < names_.size(); ++i) {
                if (transform.child_frame_id == names_[i]) {
                  poses_[i] = transform;
                  pose_times_[i] = time_;
                }
              }
            }
          });
      clock_sub_ = create_subscription<rosgraph_msgs::msg::Clock>(
          "/clock", rclcpp::QoS(1).best_effort(),
          [this](rosgraph_msgs::msg::Clock::ConstSharedPtr msg) {
            double time = msg->clock.sec + msg->clock.nanosec * 1e-9;
            if (time < time_) {
              pose_times_.fill(-1.0);
              directions_ = {1.0, -1.0};
            }
            time_ = time;
            if (time < last_update_ || time - last_update_ >= 0.05) {
              update(msg->clock);
              last_update_ = time;
            }
          });
    }

  private:
    const std::array<std::string, 2> names_{"moving_obstacle_top", "moving_obstacle_bottom"};
    std::array<double, 2> speeds_{};
    std::array<double, 2> directions_{1.0, -1.0};
    std::array<double, 2> pose_times_{-1.0, -1.0};
    std::array<geometry_msgs::msg::TransformStamped, 2> poses_;
    std::array<rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr, 2> publishers_;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr poses_sub_;
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
    double time_ = 0.0;
    double last_update_ = -1.0;

    void update(const builtin_interfaces::msg::Time& stamp)
    {
      tf2_msgs::msg::TFMessage transforms;
      for (std::size_t i = 0; i < names_.size(); ++i) {
        geometry_msgs::msg::Twist command;
        if (pose_times_[i] >= 0.0 && time_ - pose_times_[i] < 0.5) {
          double x = poses_[i].transform.translation.x;
          if (x >= 12.0) {
            directions_[i] = -1.0;
          } else if (x <= -12.0) {
            directions_[i] = 1.0;
          }
          command.linear.x = speeds_[i] * directions_[i];
          auto transform = poses_[i];
          transform.header.stamp = stamp;
          transform.header.frame_id = "sim_world";
          transform.child_frame_id = names_[i] + "/body";
          transforms.transforms.push_back(transform);
        }
        publishers_[i]->publish(command);
      }
      tf_pub_->publish(transforms);
    }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MovingObstaclesNode>());
  rclcpp::shutdown();
  return 0;
}
