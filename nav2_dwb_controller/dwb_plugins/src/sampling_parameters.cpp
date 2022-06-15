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

#include "dwb_plugins/sampling_parameters.hpp"

#include <memory>
#include <string>

#include "nav2_util/node_utils.hpp"
#include "nav_2d_utils/parameters.hpp"

using nav2_util::declare_parameter_if_not_declared;
using nav_2d_utils::moveDeprecatedParameter;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace dwb_plugins {

SamplingParamHandler::SamplingParamHandler() {
  sampling_params_.store(new SamplingParameters);
}

SamplingParamHandler::~SamplingParamHandler() {
  delete sampling_params_.load();
}

void SamplingParamHandler::initialize(
    const nav2_util::LifecycleNode::SharedPtr &nh,
    const std::string &plugin_name) {
  plugin_name_ = plugin_name;

  declare_parameter_if_not_declared(nh, plugin_name + ".vx_samples",
                                    rclcpp::ParameterValue(20));
  declare_parameter_if_not_declared(nh, plugin_name + ".vy_samples",
                                    rclcpp::ParameterValue(5));
  declare_parameter_if_not_declared(nh, plugin_name + ".vtheta_samples",
                                    rclcpp::ParameterValue(20));

  SamplingParameters sampling_params;

  nh->get_parameter(plugin_name + ".vx_samples", sampling_params.vx_samples_);
  nh->get_parameter(plugin_name + ".vy_samples", sampling_params.vy_samples_);
  nh->get_parameter(plugin_name + ".vtheta_samples",
                    sampling_params.vtheta_samples_);

  // Setup callback for changes to parameters.
  parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
      nh->get_node_base_interface(), nh->get_node_topics_interface(),
      nh->get_node_graph_interface(), nh->get_node_services_interface());

  parameter_event_sub_ = parameters_client_->on_parameter_event(
      std::bind(&SamplingParamHandler::on_parameter_event_callback, this, _1));

  update_sampling_params(sampling_params);
}

void SamplingParamHandler::on_parameter_event_callback(
    const rcl_interfaces::msg::ParameterEvent::SharedPtr event) {
  SamplingParameters sampling_params(*sampling_params_.load());

  for (auto &changed_parameter : event->changed_parameters) {
    const auto &type = changed_parameter.value.type;
    const auto &name = changed_parameter.name;
    const auto &value = changed_parameter.value;

    if (type == ParameterType::PARAMETER_INTEGER) {
      if (name == plugin_name_ + ".vx_samples") {
        sampling_params.vx_samples_ = value.integer_value;
      } else if (name == plugin_name_ + ".vy_samples") {
        sampling_params.vy_samples_ = value.integer_value;
      } else if (name == plugin_name_ + ".vtheta_samples") {
        sampling_params.vtheta_samples_ = value.integer_value;
      }
    }
  }
  update_sampling_params(sampling_params);
}

void SamplingParamHandler::update_sampling_params(
    SamplingParameters sampling_params) {
  delete sampling_params_.load();
  sampling_params_.store(new SamplingParameters(sampling_params));
}

} // namespace dwb_plugins
