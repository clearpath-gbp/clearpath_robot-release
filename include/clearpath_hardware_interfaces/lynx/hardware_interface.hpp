/**
 *
 *  \file
 *  \brief      Lynx Motor hardware interface class
 *  \author     Luis Camero <lcamero@clearpathrobotics.com>
 *  \author     Roni Kreinin <rkreinin@clearpathrobotics.com>
 *  \copyright  Copyright (c) 2024, Clearpath Robotics, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to code@clearpathrobotics.com
 *
 */
#ifndef CLEARPATH_HARDWARE_INTERFACES__LYNX_HARDWARE_INTERFACE_HPP_
#define CLEARPATH_HARDWARE_INTERFACES__LYNX_HARDWARE_INTERFACE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "clearpath_motor_msgs/msg/lynx_feedback.hpp"
#include "clearpath_motor_msgs/msg/lynx_multi_feedback.hpp"
#include "clearpath_motor_msgs/msg/lynx_motor_protection.hpp"
#include "clearpath_motor_msgs/msg/lynx_system_protection.hpp"

namespace clearpath_hardware_interfaces
{

class LynxHardwareInterface
: public rclcpp::Node
{
  public:
  explicit LynxHardwareInterface(std::string node_name);

  void drive_command(const sensor_msgs::msg::JointState msg);

  bool has_new_feedback();
  void feedback_callback(const clearpath_motor_msgs::msg::LynxMultiFeedback::SharedPtr msg);
  clearpath_motor_msgs::msg::LynxMultiFeedback get_feedback();

  void protection_callback(const clearpath_motor_msgs::msg::LynxSystemProtection::SharedPtr msg);
  clearpath_motor_msgs::msg::LynxSystemProtection get_protection();

  private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_cmd_;
  rclcpp::Subscription<clearpath_motor_msgs::msg::LynxMultiFeedback>::SharedPtr sub_feedback_;
  rclcpp::Subscription<clearpath_motor_msgs::msg::LynxSystemProtection>::SharedPtr sub_protection_;

  clearpath_motor_msgs::msg::LynxMultiFeedback feedback_;
  std::atomic_bool has_feedback_;

  clearpath_motor_msgs::msg::LynxSystemProtection protection_;
};

} // namespace clearpath_hardware_interfaces

#endif // CLEARPATH_HARDWARE_INTERFACES__LYNX_HARDWARE_INTERFACE_HPP_
