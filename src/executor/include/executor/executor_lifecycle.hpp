#pragma once

#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp/timer.hpp"
#include "std_msgs/msg/string.hpp"

namespace executor {

using LifecycleCbReturn =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class Executor : public rclcpp_lifecycle::LifecycleNode {
 public:
  Executor(const std::string& node_name, const std::string& ns,
           rclcpp::NodeOptions options=rclcpp::NodeOptions());

 protected:
  virtual LifecycleCbReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

  virtual LifecycleCbReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  virtual LifecycleCbReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  virtual LifecycleCbReturn on_shutdown(
      const rclcpp_lifecycle::State& previous_state) override;

  virtual LifecycleCbReturn on_error(
      const rclcpp_lifecycle::State& previous_state) override;

  virtual LifecycleCbReturn on_cleanup(
      const rclcpp_lifecycle::State& previous_state) override;
    
  virtual void publish_string();

  virtual void publish_vel();

 private:
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr
      vel_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr
      text_publisher_;
  rclcpp::TimerBase::SharedPtr vel_timer_;
  rclcpp::TimerBase::SharedPtr text_timer_;
};

}  // namespace executor
