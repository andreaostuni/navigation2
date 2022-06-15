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

#include "dwb_plugins/xy_theta_iterator.hpp"

#include <cmath>
#include <memory>
#include <string>

#include "nav2_util/node_utils.hpp"
#include "nav_2d_utils/parameters.hpp"

#define EPSILON 1E-5

namespace dwb_plugins {
void XYThetaIterator::initialize(KinematicsHandler::Ptr kinematics,
                                 SamplingParamHandler::Ptr sampling_params) {
  kinematics_handler_ = kinematics;
  sampling_params_handler_ = sampling_params;
}

void XYThetaIterator::startNewIteration(
    const nav_2d_msgs::msg::Twist2D &current_velocity, double dt) {
  KinematicParameters kinematics = kinematics_handler_->getKinematics();
  SamplingParameters sampling_params =
      sampling_params_handler_->getSamplingParams();
  x_it_ = std::make_shared<OneDVelocityIterator>(
      current_velocity.x, kinematics.getMinX(), kinematics.getMaxX(),
      kinematics.getAccX(), kinematics.getDecelX(), dt,
      sampling_params.getSamplesVX());
  y_it_ = std::make_shared<OneDVelocityIterator>(
      current_velocity.y, kinematics.getMinY(), kinematics.getMaxY(),
      kinematics.getAccY(), kinematics.getDecelY(), dt,
      sampling_params.getSamplesVY());
  th_it_ = std::make_shared<OneDVelocityIterator>(
      current_velocity.theta, kinematics.getMinTheta(),
      kinematics.getMaxTheta(), kinematics.getAccTheta(),
      kinematics.getDecelTheta(), dt, sampling_params.getSamplesVTheta());
  if (!isValidVelocity()) {
    iterateToValidVelocity();
  }
}

bool XYThetaIterator::isValidSpeed(double x, double y, double theta) {
  KinematicParameters kinematics = kinematics_handler_->getKinematics();
  double vmag_sq = x * x + y * y;
  if (kinematics.getMaxSpeedXY() >= 0.0 &&
      vmag_sq > kinematics.getMaxSpeedXY_SQ() + EPSILON) {
    return false;
  }
  if (kinematics.getMinSpeedXY() >= 0.0 &&
      vmag_sq + EPSILON < kinematics.getMinSpeedXY_SQ() &&
      kinematics.getMinSpeedTheta() >= 0.0 &&
      fabs(theta) + EPSILON < kinematics.getMinSpeedTheta()) {
    return false;
  }
  if (vmag_sq == 0.0 && th_it_->getVelocity() == 0.0) {
    return false;
  }
  return true;
}

bool XYThetaIterator::isValidVelocity() {
  return isValidSpeed(x_it_->getVelocity(), y_it_->getVelocity(),
                      th_it_->getVelocity());
}

bool XYThetaIterator::hasMoreTwists() { return x_it_ && !x_it_->isFinished(); }

nav_2d_msgs::msg::Twist2D XYThetaIterator::nextTwist() {
  nav_2d_msgs::msg::Twist2D velocity;
  velocity.x = x_it_->getVelocity();
  velocity.y = y_it_->getVelocity();
  velocity.theta = th_it_->getVelocity();

  iterateToValidVelocity();

  return velocity;
}

void XYThetaIterator::iterateToValidVelocity() {
  bool valid = false;
  while (!valid && hasMoreTwists()) {
    ++(*th_it_);
    if (th_it_->isFinished()) {
      th_it_->reset();
      ++(*y_it_);
      if (y_it_->isFinished()) {
        y_it_->reset();
        ++(*x_it_);
      }
    }
    valid = isValidVelocity();
  }
}

} // namespace dwb_plugins
