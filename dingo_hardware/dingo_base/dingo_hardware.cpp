/**
 *  \file       dingo_hardware.cpp
 *  \brief      Class representing Dingo hardware
 *  \copyright  Copyright (c) 2020, Clearpath Robotics, Inc.
 *
 * Software License Agreement (BSD)
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
 */

#include <boost/assign.hpp>
#include <vector>
#include <string>

#include "puma_motor_driver/driver.h"
#include "puma_motor_msg/Status.h"

#include "dingo_base/dingo_hardware.h"

namespace dingo_base
{

  DingoHardware::DingoHardware(puma_motor_driver::Gateway &gateway, bool &dingo_omni) : gateway_(gateway),
                                                                                        active_(false)
  {
    gear_ratio_ = 24;
    encoder_cpr_, 10;
    flip_motor_direction_ = false;
    gain_p_ = 0.025;
    gain_i_ = 0.005;
    gain_d_ = 0.0;

    // Set up the wheels: differs for Dingo-D vs Dingo-O
    std::vector<std::string> joint_names;
    std::vector<uint8_t> joint_can_ids;
    std::vector<float> joint_directions;
    if (!dingo_omni)
    {
      joint_names.assign({"left_wheel", "right_wheel"});
      joint_can_ids.assign({2, 3});
      joint_directions.assign({1, -1});
    }
    else
    {
      joint_names.assign({"front_left_wheel", "front_right_wheel",
                          "rear_left_wheel", "rear_right_wheel"});
      joint_can_ids.assign({2, 3, 4, 5});
      joint_directions.assign({1, -1, 1, -1});
    }

    // Flip the motor direction if needed
    if (flip_motor_direction_)
    {
      for (std::size_t i = 0; i < joint_directions.size(); i++)
      {
        joint_directions[i] *= -1;
      }
    }

    for (uint8_t i = 0; i < joint_names.size(); i++)
    {
      puma_motor_driver::Driver driver(gateway_, joint_can_ids[i], joint_names[i]);
      driver.clearMsgCache();
      driver.setEncoderCPR(encoder_cpr_);
      driver.setGearRatio(gear_ratio_ * joint_directions[i]);
      driver.setMode(puma_motor_msgs::Status::MODE_SPEED, gain_p_, gain_i_, gain_d_);
      drivers_.push_back(driver);
    }
  }

  void DingoHardware::init()
  {
    while (!connectIfNotConnected())
    {
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }

  bool DingoHardware::connectIfNotConnected()
  {
    if (!gateway_.isConnected())
    {
      if (!gateway_.connect())
      {
        ROS_ERROR("Error connecting to motor driver gateway. Retrying in 1 second.");
        return false;
      }
      else
      {
        ROS_INFO("Connection to motor driver gateway successful.");
      }
    }
    return true;
  }

  void DingoHardware::configure()
  {
    for (auto &driver : drivers_)
    {
      driver.configureParams();
    }
  }

  void DingoHardware::verify()
  {
    for (auto &driver : drivers_)
    {
      driver.verifyParams();
    }
  }

  bool DingoHardware::areAllDriversActive()
  {
    for (auto &driver : drivers_)
    {
      if (!driver.isConfigured())
      {
        return false;
      }
    }
    return true;
  }

  bool DingoHardware::isActive()
  {
    if (active_ == false && this->areAllDriversActive())
    {
      active_ = true;
      ROS_INFO("Dingo Hardware Active");
    }
    else if (!this->areAllDriversActive() && active_ == true)
    {
      active_ = false;
    }

    return active_;
  }

  void DingoHardware::powerHasNotReset()
  {
    // Checks to see if power flag has been reset for each driver
    for (auto &driver : drivers_)
    {
      if (driver.lastPower() != 0)
      {
        active_ = false;
        ROS_WARN("There was a power reset on Dev: %d, will reconfigure all drivers.", driver.deviceNumber());
        for (auto &driver : drivers_)
        {
          driver.resetConfiguration();
        }
      }
    }
  }

  bool DingoHardware::inReset()
  {
    return !active_;
  }

  void DingoHardware::requestData()
  {
    for (auto &driver : drivers_)
    {
      driver.requestFeedbackPowerState();
    }
  }

  void DingoHardware::updateJointsFromHardware()
  {
    uint8_t index = 0;
    for (auto &driver : drivers_)
    {
      Joint *f = &joints_[index];
      f->effort = driver.lastCurrent();
      f->position = driver.lastPosition();
      f->velocity = driver.lastSpeed();
      index++;
    }
  }

  void DingoHardware::command()
  {
    uint8_t i = 0;
    for (auto &driver : drivers_)
    {
      driver.commandSpeed(joints_[i].velocity_command);
      i++;
    }
  }

  void DingoHardware::canRead()
  {
    puma_motor_driver::Message recv_msg;
    while (gateway_.recv(&recv_msg))
    {
      for (auto &driver : drivers_)
      {
        driver.processMessage(recv_msg);
      }
    }
  }

} // namespace dingo_base
