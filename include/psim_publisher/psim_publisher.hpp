/*
 * Copyright 2021 Tier IV, Inc. All rights reserved.
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

#ifndef PSIM_PUBLISHER_PSIM_PUBLISHER_HPP_
#define PSIM_PUBLISHER_PSIM_PUBLISHER_HPP_

/**
 * @file  psim_publisher.hpp
 * @brief Psim publisher class
 */

#include "autoware_system_msgs/msg/autoware_state.hpp"
#include "autoware_vehicle_msgs/msg/engage.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

#include "rclcpp/rclcpp.hpp"

class PsimPublisher : public rclcpp::Node
{
public:
  /**
   * @brief constructor
   */
  PsimPublisher();

protected:
  /**
   * @brief timer callback
   */
  void onTimer();

  void callbackStatus(const autoware_system_msgs::msg::AutowareState::ConstSharedPtr msg);

  rclcpp::TimerBase::SharedPtr timer_;                                                          //!< @brief timer
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_initialpose_; //!< @brief publiser
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_;                      //!< @brief publiser
  rclcpp::Publisher<autoware_vehicle_msgs::msg::Engage>::SharedPtr pub_engage_;                 //!< @brief publiser
  rclcpp::Subscription<autoware_system_msgs::msg::AutowareState>::SharedPtr sub_state_;         //!< @brief subscriber

  geometry_msgs::msg::PoseWithCovarianceStamped initialpose_;
  geometry_msgs::msg::PoseStamped goal_;
  autoware_system_msgs::msg::AutowareState::ConstSharedPtr state_;
  double start_;
  double duration_;
};

#endif  // PSIM_PUBLISHER_PSIM_PUBLISHER_HPP_
