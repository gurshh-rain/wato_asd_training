#include <cmath>
#include <limits>

#include "control_core.hpp"
#include "gtest/gtest.h"

TEST(ControlCore, RotatesBeforeDrivingTowardTargetBehindRobot)
{
  robot::ControlCore control(rclcpp::get_logger("test"), 0.6, 0.2, 0.5,
                             1.5, 0.65, 1.8);
  nav_msgs::msg::Odometry odom;
  odom.pose.pose.orientation.w = 1.0;
  nav_msgs::msg::Path path;
  geometry_msgs::msg::PoseStamped target;
  target.pose.position.x = -2.0;
  target.pose.orientation.w = 1.0;
  path.poses.push_back(target);
  auto command = control.computeCommand(path, odom);
  EXPECT_DOUBLE_EQ(command.linear.x, 0.0);
  EXPECT_GT(std::abs(command.angular.z), 0.0);
}

TEST(ControlCore, RotatesBeforeEnteringSharpCorner)
{
  robot::ControlCore control(rclcpp::get_logger("test"), 0.4, 0.2, 1.0,
                             1.5, 0.4, 1.8);
  nav_msgs::msg::Odometry odom;
  odom.pose.pose.orientation.w = 1.0;
  nav_msgs::msg::Path path;
  geometry_msgs::msg::PoseStamped target;
  target.pose.position.x = 2.0 * std::cos(0.5);
  target.pose.position.y = 2.0 * std::sin(0.5);
  target.pose.orientation.w = 1.0;
  path.poses.push_back(target);
  auto command = control.computeCommand(path, odom);
  EXPECT_DOUBLE_EQ(command.linear.x, 0.0);
  EXPECT_GT(command.angular.z, 0.0);
}

TEST(ControlCore, DrivesTowardAlignedTarget)
{
  robot::ControlCore control(rclcpp::get_logger("test"), 0.6, 0.2, 0.5,
                             1.5, 0.65, 1.8);
  nav_msgs::msg::Odometry odom;
  odom.pose.pose.orientation.w = 1.0;
  nav_msgs::msg::Path path;
  geometry_msgs::msg::PoseStamped target;
  target.pose.position.x = 2.0;
  target.pose.orientation.w = 1.0;
  path.poses.push_back(target);
  auto command = control.computeCommand(path, odom);
  EXPECT_GT(command.linear.x, 0.0);
  EXPECT_NEAR(command.angular.z, 0.0, 1e-9);
}

namespace
{
nav_msgs::msg::Path straightPath()
{
  nav_msgs::msg::Path path;
  for (int i = 0; i <= 100; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = i * 0.1;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return path;
}

sensor_msgs::msg::LaserScan clearScan()
{
  sensor_msgs::msg::LaserScan scan;
  scan.angle_min = -3.141592653589793;
  scan.angle_increment = 3.141592653589793 / 180.0;
  scan.range_min = 0.08;
  scan.range_max = 20.0;
  scan.ranges.assign(361, std::numeric_limits<float>::infinity());
  return scan;
}

robot::ObstaclePrediction approaching(double x, double y, double vx, double vy)
{
  robot::ObstaclePrediction track;
  for (int i = 0; i <= 25; ++i) {
    geometry_msgs::msg::Point p;
    p.x = x + vx * i * track.step;
    p.y = y + vy * i * track.step;
    track.points.push_back(p);
  }
  return track;
}
}

TEST(ControlCore, RejectsStraightHeadOnCollisionAndSelectsLateralMotion)
{
  robot::ControlCore control(rclcpp::get_logger("test"), .4, .2, 1., 1.5, .4, 1.8);
  nav_msgs::msg::Odometry odom;
  odom.pose.pose.orientation.w = 1.0;
  auto scan = clearScan();
  std::vector<robot::ObstaclePrediction> tracks{approaching(6., 0., -1., 0.)};
  geometry_msgs::msg::Twist straight;
  straight.linear.x = 1.;
  EXPECT_LT(control.motionClearance(straight, odom, scan, tracks), 0.0);
  auto decision = control.selectCommand(straightPath(), odom, scan, tracks);
  EXPECT_GT(std::abs(decision.command.angular.z), 0.05);
  EXPECT_GT(control.motionClearance(decision.command, odom, scan, tracks), 0.0);
}

TEST(ControlCore, DetectsCollisionWithStoppedRobot)
{
  robot::ControlCore control(rclcpp::get_logger("test"), .4, .2, 1., 1.5, .4, 1.8);
  nav_msgs::msg::Odometry odom;
  odom.pose.pose.orientation.w = 1.0;
  std::vector<robot::ObstaclePrediction> tracks{approaching(3., 0., -1., 0.)};
  EXPECT_LT(control.motionClearance(geometry_msgs::msg::Twist(), odom, clearScan(), tracks), 0.0);
}

TEST(ControlCore, ChecksRearBodyDuringRotation)
{
  robot::ControlCore control(rclcpp::get_logger("test"), .4, .2, 1., 1.5, .4, 1.8);
  nav_msgs::msg::Odometry odom;
  odom.pose.pose.orientation.w = 1.0;
  std::vector<robot::ObstaclePrediction> tracks{approaching(-1.3, 1.5, 0., 0.)};
  tracks[0].radius = .15;
  geometry_msgs::msg::Twist rotation;
  rotation.angular.z = 1.;
  EXPECT_GT(control.motionClearance(geometry_msgs::msg::Twist(), odom, clearScan(), tracks), 0.0);
  EXPECT_LT(control.motionClearance(rotation, odom, clearScan(), tracks), 0.0);
}

TEST(ControlCore, StalePredictionDoesNotPermitMotion)
{
  robot::ControlCore control(rclcpp::get_logger("test"), .4, .2, 1., 1.5, .4, 1.8);
  nav_msgs::msg::Odometry odom;
  odom.pose.pose.orientation.w = 1.0;
  auto track = approaching(6., 0., -1., 0.);
  track.age = 2.0;
  auto decision = control.selectCommand(straightPath(), odom, clearScan(), {track});
  EXPECT_DOUBLE_EQ(decision.command.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(decision.command.angular.z, 0.0);
  EXPECT_EQ(decision.state, "NO_SAFE_MOTION");
}

TEST(ControlCore, RepeatedHeadOnControlPassesBesideObstacle)
{
  robot::ControlCore control(rclcpp::get_logger("test"), .4, .2, 1., 1.5, .4, 1.8);
  nav_msgs::msg::Odometry odom;
  double x = -1.3, y = 0.0, yaw = 0.0;
  double largest_lateral = 0.0;
  for (int i = 0; i < 120; ++i) {
    odom.pose.pose.position.x = x + 1.3 * std::cos(yaw);
    odom.pose.pose.position.y = y + 1.3 * std::sin(yaw);
    odom.pose.pose.orientation.z = std::sin(yaw / 2.0);
    odom.pose.pose.orientation.w = std::cos(yaw / 2.0);
    auto track = approaching(6.0 - i * .1, 0., -1., 0.);
    auto decision = control.selectCommand(straightPath(), odom, clearScan(), {track});
    ASSERT_GT(control.motionClearance(decision.command, odom, clearScan(), {track}), 0.0);
    double w = decision.command.angular.z;
    double v = decision.command.linear.x;
    if (std::abs(w) > 1e-6) {
      x += v / w * (std::sin(yaw + w * .1) - std::sin(yaw));
      y += v / w * (std::cos(yaw) - std::cos(yaw + w * .1));
    } else {
      x += v * .1 * std::cos(yaw);
      y += v * .1 * std::sin(yaw);
    }
    yaw += w * .1;
    largest_lateral = std::max(largest_lateral, std::abs(y));
  }
  EXPECT_GT(largest_lateral, 1.5);
  EXPECT_GT(x, 1.0);
}

TEST(ControlCore, CrossingMotionIsCheckedAtMatchingTimes)
{
  robot::ControlCore control(rclcpp::get_logger("test"), .4, .2, 1., 1.5, .4, 1.8);
  nav_msgs::msg::Odometry odom;
  odom.pose.pose.orientation.w = 1.0;
  geometry_msgs::msg::Twist straight;
  straight.linear.x = 1.;
  EXPECT_LT(control.motionClearance(straight, odom, clearScan(),
      {approaching(2., -2., 0., 1.)}), 0.0);
  EXPECT_GT(control.motionClearance(straight, odom, clearScan(),
      {approaching(2., 3., 0., 1.)}), 0.0);
}

