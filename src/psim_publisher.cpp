/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file psim_publisher.cpp
 * @brief psim publisher class
 */

#include "psim_publisher/psim_publisher.hpp"
#include <functional>

using autoware_system_msgs::msg::AutowareState;
using std::placeholders::_1;

PsimPublisher::PsimPublisher() : Node("psim_publisher"), start_(0)
{
  initialpose_.header.frame_id = declare_parameter("frame_id", "viewer");
  initialpose_.pose.pose.position.x = declare_parameter("pose_x", 0.0);
  initialpose_.pose.pose.position.y = declare_parameter("pose_y", 0.0);
  initialpose_.pose.pose.position.z = declare_parameter("pose_z", 0.0);
  initialpose_.pose.pose.orientation.x = declare_parameter("orie_x", 0.0);
  initialpose_.pose.pose.orientation.y = declare_parameter("orie_y", 0.0);
  initialpose_.pose.pose.orientation.z = declare_parameter("orie_z", 0.0);
  initialpose_.pose.pose.orientation.w = declare_parameter("orie_w", 0.0);
  initialpose_.pose.covariance[0] = declare_parameter("cov_x", 0.25);
  initialpose_.pose.covariance[7] = declare_parameter("cov_y", 0.25);
  initialpose_.pose.covariance[14] = declare_parameter("cov_z", 0.0);
  initialpose_.pose.covariance[35] = declare_parameter("cov_yaw", (M_PI / 12.0) * (M_PI / 12.0));

  goal_.header.frame_id = initialpose_.header.frame_id;
  goal_.pose.position.x = declare_parameter("goal_pose_x", 0.0);
  goal_.pose.position.y = declare_parameter("goal_pose_y", 0.0);
  goal_.pose.position.z = declare_parameter("goal_pose_z", 0.0);
  goal_.pose.orientation.x = declare_parameter("goal_orie_x", 0.0);
  goal_.pose.orientation.y = declare_parameter("goal_orie_y", 0.0);
  goal_.pose.orientation.z = declare_parameter("goal_orie_z", 0.0);
  goal_.pose.orientation.w = declare_parameter("goal_orie_w", 0.0);

  duration_ = declare_parameter("duration", 0.0);

  // Timer
  if (duration_ > 0.0) {
    auto timer_callback = std::bind(&PsimPublisher::onTimer, this);
    auto period =
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0));

    timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
      this->get_clock(), period, std::move(timer_callback),
      this->get_node_base_interface()->get_context());
    this->get_node_timers_interface()->add_timer(timer_, nullptr);
  }

  pub_initialpose_ =
    this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 1);
  pub_goal_ =
    this->create_publisher<geometry_msgs::msg::PoseStamped>("/planning/mission_planning/goal", 1);
  pub_engage_ = this->create_publisher<autoware_vehicle_msgs::msg::Engage>("/autoware/engage", 1);

  sub_state_ = this->create_subscription<autoware_system_msgs::msg::AutowareState>(
    "/autoware/state", 1, std::bind(&PsimPublisher::callbackStatus, this, _1));

  rclcpp::Clock system_clock(RCL_SYSTEM_TIME);
  start_ = system_clock.now().seconds();
}

void PsimPublisher::onTimer()
{
  rclcpp::Clock system_clock(RCL_SYSTEM_TIME);
  if (system_clock.now().seconds() - start_ >= duration_) {
    RCLCPP_INFO(get_logger(), "Shutdown.");
    rclcpp::shutdown();
  }
}

void PsimPublisher::callbackStatus(
  const autoware_system_msgs::msg::AutowareState::ConstSharedPtr msg)
{
  if (state_) {
    if (state_->state == msg->state) return;
  }

  state_ = msg;

  RCLCPP_INFO(get_logger(), "%s", state_->state.c_str());

  if (state_->state == AutowareState::INITIALIZING_VEHICLE) {
    pub_initialpose_->publish(initialpose_);
  }
  if (state_->state == AutowareState::WAITING_FOR_ROUTE) {
    pub_goal_->publish(goal_);
  }
  if (state_->state == AutowareState::WAITING_FOR_ENGAGE) {
    autoware_vehicle_msgs::msg::Engage msg;
    msg.engage = true;
    pub_engage_->publish(msg);
  }
  if (state_->state == AutowareState::ARRIVAL_GOAL) {
    pub_initialpose_->publish(initialpose_);
  }
}
