/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef DWB_PLUGINS__SAMPLING_PARAMETERS_HPP_
#define DWB_PLUGINS__SAMPLING_PARAMETERS_HPP_

#include <memory>
#include <string>

#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dwb_plugins {

/**
 * @struct SamplingParameters
 * @brief A struct containing one representation of the parameters used to
 * sample the velocities to generate the trajectories
 */
struct SamplingParameters {
  friend class SamplingParamHandler;

  inline int getSamplesVX() { return vx_samples_; }
  inline int getSamplesVY() { return vy_samples_; }
  inline int getSamplesVTheta() { return vtheta_samples_; }

protected:
  // For parameter descriptions, see cfg/SamplingParams.cfg
  int vx_samples_{0};
  int vy_samples_{0};
  int vtheta_samples_{0};
};

/**
 * @class KinematicsHandler
 * @brief A class managing the representation of the robot's kinematics
 */
class SamplingParamHandler {
public:
  SamplingParamHandler();
  ~SamplingParamHandler();
  void initialize(const nav2_util::LifecycleNode::SharedPtr &nh,
                  const std::string &plugin_name);

  inline SamplingParameters getSamplingParams() {
    return *sampling_params_.load();
  }

  using Ptr = std::shared_ptr<SamplingParamHandler>;

protected:
  std::atomic<SamplingParameters *> sampling_params_;

  // Subscription for parameter change
  rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr
      parameter_event_sub_;
  void on_parameter_event_callback(
      const rcl_interfaces::msg::ParameterEvent::SharedPtr event);
  void update_sampling_params(SamplingParameters sampling_params);
  std::string plugin_name_;
};

} // namespace dwb_plugins

#endif // DWB_PLUGINS__SAMPLING_PARAMETERS_HPP_
