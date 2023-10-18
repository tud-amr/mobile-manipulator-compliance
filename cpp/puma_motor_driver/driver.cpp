/**
Software License Agreement (BSD)

\authors   Mike Purvis <mpurvis@clearpathrobotics.com>
\copyright Copyright (c) 2015, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "puma_motor_driver/driver.h"
#include "puma_motor_driver/gateway.h"
#include "puma_motor_driver/message.h"
#include "puma_motor_msg/Status.h"

#include <string>
#include <cstring>
#include <math.h>
#include <stdexcept>
#include <vector>
#include <map>
#include <iostream>
#include <thread>

namespace puma_motor_driver
{

  namespace ConfigurationStates
  {
    enum ConfigurationState
    {
      Unknown = -1,
      Initializing,
      PowerFlag,
      EncoderPosRef,
      EncoderSpdRef,
      EncoderCounts,
      ClosedLoop,
      ControlMode,
      PGain,
      IGain,
      DGain,
      VerifiedParameters,
      Configured
    };
  } // namespace ConfigurationStates
  typedef ConfigurationStates::ConfigurationState ConfigurationState;

  Driver::Driver(Gateway &gateway, const uint8_t &device_number, const std::string &device_name)
      : gateway_(gateway), device_number_(device_number), device_name_(device_name), encoder_cpr_(10), gear_ratio_(24)
  {
    pid_apis_ = {
        {"CurP", LM_API_ICTRL_PC},
        {"CurI", LM_API_ICTRL_IC},
        {"CurD", LM_API_ICTRL_DC},
        {"PosP", LM_API_POS_PC},
        {"PosI", LM_API_POS_IC},
        {"PosD", LM_API_POS_DC},
        {"SpdP", LM_API_SPD_PC},
        {"SpdI", LM_API_SPD_IC},
        {"SpdD", LM_API_SPD_DC},
    };
  }

  void Driver::processMessage(const Message &received_msg)
  {
    // If it's not our message, jump out.
    if (received_msg.getDeviceNumber() != device_number_)
      return;

    // If there's no data then this is a request message, jump out.
    if (received_msg.len == 0)
      return;

    Field *field = nullptr;
    if ((received_msg.getApi() & CAN_MSGID_API_M & CAN_API_MC_CFG) == CAN_API_MC_CFG)
    {
      field = cfgFieldForMessage(received_msg);
    }
    else if ((received_msg.getApi() & CAN_MSGID_API_M & CAN_API_MC_STATUS) == CAN_API_MC_STATUS)
    {
      field = statusFieldForMessage(received_msg);
    }
    else if ((received_msg.getApi() & CAN_MSGID_API_M & CAN_API_MC_ICTRL) == CAN_API_MC_ICTRL)
    {
      field = ictrlFieldForMessage(received_msg);
    }
    else if ((received_msg.getApi() & CAN_MSGID_API_M & CAN_API_MC_POS) == CAN_API_MC_POS)
    {
      field = posFieldForMessage(received_msg);
    }
    else if ((received_msg.getApi() & CAN_MSGID_API_M & CAN_API_MC_VCOMP) == CAN_API_MC_VCOMP)
    {
      field = vcompFieldForMessage(received_msg);
    }
    else if ((received_msg.getApi() & CAN_MSGID_API_M & CAN_API_MC_SPD) == CAN_API_MC_SPD)
    {
      field = spdFieldForMessage(received_msg);
    }
    else if ((received_msg.getApi() & CAN_MSGID_API_M & CAN_API_MC_VOLTAGE) == CAN_API_MC_VOLTAGE)
    {
      field = voltageFieldForMessage(received_msg);
    }

    if (!field)
      return;

    // Copy the received data and mark that field as received.
    std::memcpy(field->data, received_msg.data, received_msg.len);
    field->received = true;
  }

  double Driver::radPerSecToRpm() const
  {
    return ((60 * gear_ratio_) / (2 * M_PI));
  }

  void Driver::sendUint8(const uint32_t id, const uint8_t value)
  {
    Message msg;
    msg.id = id;
    msg.len = 1;
    std::memcpy(msg.data, &value, msg.len);
    gateway_.queue(msg);
  }

  void Driver::sendUint16(const uint32_t id, const uint16_t value)
  {
    Message msg;
    msg.id = id;
    msg.len = 2;
    std::memcpy(msg.data, &value, msg.len);
    gateway_.queue(msg);
  }

  void Driver::sendFixed8x8(const uint32_t id, const float value)
  {
    Message msg;
    msg.id = id;
    msg.len = 2;
    int16_t output_value = static_cast<int16_t>(static_cast<float>(1 << 8) * value);
    std::memcpy(msg.data, &output_value, msg.len);
    gateway_.queue(msg);
  }

  void Driver::sendFixed16x16(const uint32_t id, const double value)
  {
    Message msg;
    msg.id = id;
    msg.len = 4;
    int32_t output_value = static_cast<int32_t>(static_cast<double>((1 << 16) * value));
    std::memcpy(msg.data, &output_value, msg.len);
    gateway_.queue(msg);
  }

  bool Driver::verifyRaw16x16(const uint8_t *received, const double expected)
  {
    uint8_t data[4];
    int32_t output_value = static_cast<int32_t>(static_cast<double>((1 << 16) * expected));
    std::memcpy(data, &output_value, 4);
    for (uint8_t i = 0; i < 4; i++)
    {
      if (*received != data[i])
      {
        return false;
      }
      received++;
    }
    return true;
  }

  bool Driver::verifyRaw8x8(const uint8_t *received, const float expected)
  {
    uint8_t data[2];
    int32_t output_value = static_cast<int32_t>(static_cast<float>((1 << 8) * expected));
    std::memcpy(data, &output_value, 2);
    for (uint8_t i = 0; i < 2; i++)
    {
      if (*received != data[i])
      {
        return false;
      }
      received++;
    }
    return true;
  }

  void Driver::setEncoderCPR(const uint16_t encoder_cpr)
  {
    encoder_cpr_ = encoder_cpr;
  }

  void Driver::setGearRatio(const float gear_ratio)
  {
    gear_ratio_ = gear_ratio;
  }

  void Driver::commandDutyCycle(const float cmd)
  {
    sendFixed8x8((LM_API_VOLT_SET | device_number_), cmd * 128.0);
  }

  void Driver::commandSpeed(const double cmd)
  {
    // Converting from rad/s to RPM through the gearbox.
    sendFixed16x16((LM_API_SPD_SET | device_number_), (cmd * radPerSecToRpm()));
  }

  void Driver::commandPosition(const double cmd)
  {
    sendFixed16x16((LM_API_POS_SET | device_number_), cmd * (gear_ratio_ / (2 * M_PI)));
  }

  void Driver::commandCurrent(const double cmd)
  {
    sendFixed8x8((LM_API_ICTRL_SET | device_number_), cmd);
  }

  void Driver::setMode(uint8_t mode)
  {
    clearMsgCache();
    bool succes = false;
    while (!succes)
    {
      while (!lastModeReceived())
      {
        gateway_.queue(Message(LM_API_STATUS_CMODE | device_number_));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      if (lastMode() == mode)
      {
        succes = true;
      }
      else
      {
        std::cout << "Trying to switch to mode: " << mode << "but currently in mode: " << lastMode() << std::endl;
        switch (mode)
        {
        case puma_motor_msgs::Status::MODE_VOLTAGE:
          gateway_.queue(Message(LM_API_VOLT_EN | device_number_));
          break;
        case puma_motor_msgs::Status::MODE_CURRENT:
          gateway_.queue(Message(LM_API_ICTRL_EN | device_number_));
          break;
        case puma_motor_msgs::Status::MODE_POSITION:
          gateway_.queue(Message(LM_API_POS_EN | device_number_));
          break;
        case puma_motor_msgs::Status::MODE_SPEED:
          gateway_.queue(Message(LM_API_SPD_EN | device_number_));
          break;
        }
        clearMsgCache();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
    std::cout << "Mode set to: " << unsigned(lastMode()) << std::endl;
  }

  void Driver::clearMsgCache()
  {
    // Set it all to zero, which will in part clear
    // the boolean flags to be false.
    memset(voltage_fields_, 0, sizeof(voltage_fields_));
    memset(spd_fields_, 0, sizeof(spd_fields_));
    memset(vcomp_fields_, 0, sizeof(vcomp_fields_));
    memset(pos_fields_, 0, sizeof(pos_fields_));
    memset(ictrl_fields_, 0, sizeof(ictrl_fields_));
    memset(status_fields_, 0, sizeof(status_fields_));
    memset(cfg_fields_, 0, sizeof(cfg_fields_));
  }

  void Driver::requestStatusMessages()
  {
    gateway_.queue(Message(LM_API_STATUS_POWER | device_number_));
  }

  void Driver::requestFeedbackMessages()
  {
    gateway_.queue(Message(LM_API_STATUS_VOLTOUT | device_number_));
    gateway_.queue(Message(LM_API_STATUS_CURRENT | device_number_));
    gateway_.queue(Message(LM_API_STATUS_POS | device_number_));
    gateway_.queue(Message(LM_API_STATUS_SPD | device_number_));
    gateway_.queue(Message(LM_API_SPD_SET | device_number_));
  }
  void Driver::requestFeedbackDutyCycle()
  {
    gateway_.queue(Message(LM_API_STATUS_VOLTOUT | device_number_));
  }

  void Driver::requestFeedbackVoltOut()
  {
    gateway_.queue(Message(LM_API_STATUS_VOUT | device_number_));
  }

  void Driver::requestFeedbackCurrent()
  {
    gateway_.queue(Message(LM_API_STATUS_CURRENT | device_number_));
  }

  void Driver::requestFeedbackPosition()
  {
    gateway_.queue(Message(LM_API_STATUS_POS | device_number_));
  }

  void Driver::requestFeedbackSpeed()
  {
    gateway_.queue(Message(LM_API_STATUS_SPD | device_number_));
  }

  void Driver::requestFeedbackPowerState()
  {
    gateway_.queue(Message(LM_API_STATUS_POWER | device_number_));
  }

  void Driver::requestFeedbackSetpoint()
  {
    gateway_.queue(Message(LM_API_ICTRL_SET | device_number_));
    gateway_.queue(Message(LM_API_POS_SET | device_number_));
    gateway_.queue(Message(LM_API_SPD_SET | device_number_));
    gateway_.queue(Message(LM_API_VOLT_SET | device_number_));
  }

  float Driver::lastDutyCycle()
  {
    Field *field = statusFieldForMessage(Message(LM_API_STATUS_VOLTOUT));
    return (field->interpretFixed8x8() / 128.0);
  }

  float Driver::lastBusVoltage()
  {
    Field *field = statusFieldForMessage(Message(LM_API_STATUS_VOLTBUS));
    return field->interpretFixed8x8();
  }

  float Driver::lastCurrent()
  {
    Field *field = statusFieldForMessage(Message(LM_API_STATUS_CURRENT));
    return field->interpretFixed8x8();
  }

  bool Driver::lastCurrentReceived()
  {
    Field *field = statusFieldForMessage(Message(LM_API_STATUS_CURRENT));
    return field->received;
  }

  double Driver::lastPosition()
  {
    Field *field = statusFieldForMessage(Message(LM_API_STATUS_POS));
    return (field->interpretFixed16x16() * ((2 * M_PI) / gear_ratio_)); // Convert rev to rad
  }

  bool Driver::lastPositionReceived()
  {
    Field *field = statusFieldForMessage(Message(LM_API_STATUS_POS));
    return field->received;
  }

  double Driver::lastSpeed()
  {
    Field *field = statusFieldForMessage(Message(LM_API_STATUS_SPD));
    return (field->interpretFixed16x16() * ((2 * M_PI) / (gear_ratio_ * 60))); // Convert RPM to rad/s
  }

  bool Driver::lastSpeedReceived()
  {
    Field *field = statusFieldForMessage(Message(LM_API_STATUS_SPD));
    return field->received;
  }

  uint8_t Driver::lastFault()
  {
    Field *field = statusFieldForMessage(Message(LM_API_STATUS_FAULT));
    return field->data[0];
  }

  uint8_t Driver::lastPower()
  {
    Field *field = statusFieldForMessage(Message(LM_API_STATUS_POWER));
    return field->data[0];
  }

  bool Driver::lastPowerReceived()
  {
    Field *field = statusFieldForMessage(Message(LM_API_STATUS_POWER));
    return field->received;
  }

  uint8_t Driver::lastMode()
  {
    Field *field = statusFieldForMessage(Message(LM_API_STATUS_CMODE));
    return field->data[0];
  }

  bool Driver::lastModeReceived()
  {
    Field *field = statusFieldForMessage(Message(LM_API_STATUS_CMODE));
    return field->received;
  }

  float Driver::lastOutVoltage()
  {
    Field *field = statusFieldForMessage(Message(LM_API_STATUS_VOUT));
    return field->interpretFixed8x8();
  }

  bool Driver::lastOutVoltageReceived()
  {
    Field *field = statusFieldForMessage(Message(LM_API_STATUS_VOUT));
    return field->received;
  }

  float Driver::lastTemperature()
  {
    Field *field = statusFieldForMessage(Message(LM_API_STATUS_TEMP));
    return field->interpretFixed8x8();
  }

  float Driver::lastAnalogInput()
  {
    Field *field = statusFieldForMessage(Message(CPR_API_STATUS_ANALOG));
    return field->interpretFixed8x8();
  }

  double Driver::statusSpeedGet()
  {
    Field *field = spdFieldForMessage(Message(LM_API_SPD_SET));
    return (field->interpretFixed16x16() * ((2 * M_PI) / (gear_ratio_ * 60))); // Convert RPM to rad/s
  }

  float Driver::statusDutyCycleGet()
  {
    Field *field = voltageFieldForMessage(Message(LM_API_VOLT_SET));
    return (field->interpretFixed8x8() / 128.0);
  }

  float Driver::statusCurrentGet()
  {
    Field *field = ictrlFieldForMessage(Message(LM_API_ICTRL_SET));
    return field->interpretFixed8x8();
  }
  double Driver::statusPositionGet()
  {
    Field *field = posFieldForMessage(Message(LM_API_POS_SET));
    return (field->interpretFixed16x16() * ((2 * M_PI) / gear_ratio_)); // Convert rev to rad
  }

  uint8_t Driver::posEncoderRef()
  {
    Field *field = posFieldForMessage(Message(LM_API_POS_REF));
    return field->data[0];
  }
  bool Driver::posEncoderRefReceived()
  {
    Field *field = posFieldForMessage(Message(LM_API_POS_REF));
    return field->received;
  }

  uint8_t Driver::spdEncoderRef()
  {
    Field *field = spdFieldForMessage(Message(LM_API_SPD_REF));
    return field->data[0];
  }
  bool Driver::spdEncoderRefReceived()
  {
    Field *field = spdFieldForMessage(Message(LM_API_SPD_REF));
    return field->received;
  }

  uint16_t Driver::encoderCounts()
  {
    Field *field = cfgFieldForMessage(Message(LM_API_CFG_ENC_LINES));
    return (static_cast<uint16_t>(field->data[0]) | static_cast<uint16_t>(field->data[1] << 8));
  }

  Driver::Field *Driver::voltageFieldForMessage(const Message &msg)
  {
    uint32_t voltage_field_index = (msg.getApi() & CAN_MSGID_API_ID_M) >> CAN_MSGID_API_S;
    return &voltage_fields_[voltage_field_index];
  }

  Driver::Field *Driver::spdFieldForMessage(const Message &msg)
  {
    uint32_t spd_field_index = (msg.getApi() & CAN_MSGID_API_ID_M) >> CAN_MSGID_API_S;
    return &spd_fields_[spd_field_index];
  }

  Driver::Field *Driver::vcompFieldForMessage(const Message &msg)
  {
    uint32_t vcomp_field_index = (msg.getApi() & CAN_MSGID_API_ID_M) >> CAN_MSGID_API_S;
    return &vcomp_fields_[vcomp_field_index];
  }

  Driver::Field *Driver::posFieldForMessage(const Message &msg)
  {
    uint32_t pos_field_index = (msg.getApi() & CAN_MSGID_API_ID_M) >> CAN_MSGID_API_S;
    return &pos_fields_[pos_field_index];
  }

  Driver::Field *Driver::ictrlFieldForMessage(const Message &msg)
  {
    uint32_t ictrl_field_index = (msg.getApi() & CAN_MSGID_API_ID_M) >> CAN_MSGID_API_S;
    return &ictrl_fields_[ictrl_field_index];
  }

  Driver::Field *Driver::statusFieldForMessage(const Message &msg)
  {
    uint32_t status_field_index = (msg.getApi() & CAN_MSGID_API_ID_M) >> CAN_MSGID_API_S;
    return &status_fields_[status_field_index];
  }

  Driver::Field *Driver::cfgFieldForMessage(const Message &msg)
  {
    uint32_t cfg_field_index = (msg.getApi() & CAN_MSGID_API_ID_M) >> CAN_MSGID_API_S;
    return &cfg_fields_[cfg_field_index];
  }

  void Driver::canRead()
  {
    puma_motor_driver::Message recv_msg;
    while (gateway_.recv(&recv_msg))
    {
      processMessage(recv_msg);
    }
  }

  // EXPTECTED CONVERSIONS:
  double Driver::expectedDoubleAfter16x16Conversion(double value)
  {
    int32_t int_value = static_cast<int32_t>(static_cast<double>((1 << 16) * value));
    return (reinterpret_cast<int32_t>(int_value)) / static_cast<double>(1 << 16);
  }

  double Driver::expectedDoubleAfter8x8Conversion(double value)
  {
    int16_t int_value = static_cast<int16_t>(static_cast<float>(1 << 8) * value);
    return (reinterpret_cast<int16_t>(int_value)) / static_cast<float>(1 << 8);
  }

  // ENCODERS:
  void Driver::setPosEncoderRef()
  {
    clearMsgCache();
    bool succes = false;
    while (!succes)
    {
      std::cout << "Wait till received" << std::endl;
      while (!posEncoderRefReceived())
      {
        gateway_.queue(Message(LM_API_POS_REF | device_number_));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      std::cout << "Received" << std::endl;
      if (posEncoderRef() == LM_REF_ENCODER)
      {
        succes = true;
      }
      else
      {
        sendUint8((LM_API_POS_REF | device_number_), LM_REF_ENCODER);
      }
    }
    std::cout << "posEncoderRef was set to: " << unsigned(posEncoderRef()) << std::endl;
  }

  void Driver::setSpdEncoderRef()
  {
    clearMsgCache();
    bool succes = false;
    while (!succes)
    {
      while (!spdEncoderRefReceived())
      {
        gateway_.queue(Message(LM_API_SPD_REF | device_number_));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      if (spdEncoderRef() == LM_REF_QUAD_ENCODER)
      {
        succes = true;
      }
      else
      {
        sendUint8((LM_API_SPD_REF | device_number_), LM_REF_QUAD_ENCODER);
      }
    }
    std::cout << "spdEncoderRef was set to: " << unsigned(spdEncoderRef()) << std::endl;
  }

  // GAIN:
  double Driver::getGain(std::string mode, std::string gain, bool clear_first)
  {
    int api;
    auto it = pid_apis_.find(mode + gain);
    if (it == pid_apis_.end())
    {
      std::cout << "Api was not found.";
      return 0;
    }
    else
    {
      api = it->second;
    }

    Field *field;
    if (mode == "Cur")
    {
      field = ictrlFieldForMessage(Message(api));
    }
    else if (mode == "Pos")
    {
      field = posFieldForMessage(Message(api));
    }
    else if (mode == "Spd")
    {
      field = spdFieldForMessage(Message(api));
    }

    if (clear_first)
    {
      clearMsgCache();
    }
    while (!(field->received))
    {
      gateway_.queue(Message(api | device_number_));
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return field->interpretFixed16x16();
  };

  void Driver::setGain(std::string mode, std::string gain, double value)
  {
    int api;
    auto it = pid_apis_.find(mode + gain);
    if (it == pid_apis_.end())
    {
      std::cout << "Api was not found.";
      return;
    }
    else
    {
      api = it->second;
    }

    bool succes = false;
    while (!succes)
    {
      double expected_value = expectedDoubleAfter16x16Conversion(value);
      double received_value = getGain(mode, gain, true);
      if (received_value == expected_value)
      {
        succes = true;
      }
      else
      {
        std::cout << "Expected " << expected_value << ", but received " << received_value << std::endl;
        sendFixed16x16((api | device_number_), value);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
    std::cout << mode + gain << " was set to " << getGain(mode, gain) << std::endl;
  }

  // INFO:
  void Driver::printPIDs()
  {
    clearMsgCache();
    std::cout << "CUR:\tP: " << getGain("Cur", "P") << "\tI: " << getGain("Cur", "I") << "\tD: " << getGain("Cur", "D") << std::endl;
    std::cout << "POS:\tP: " << getGain("Pos", "P") << "\tI: " << getGain("Pos", "I") << "\tD: " << getGain("Pos", "D") << std::endl;
    std::cout << "SPD:\tP: " << getGain("Spd", "P") << "\tI: " << getGain("Spd", "I") << "\tD: " << getGain("Spd", "D") << std::endl;
  }

  void Driver::printFeedback()
  {
    requestStatusMessages();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    requestFeedbackMessages();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    requestFeedbackSetpoint();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::cout << "power: " << unsigned(lastPower()) << " voltout: " << lastOutVoltage() << " current: " << lastCurrent() << " pos: " << lastPosition() << " spd: " << lastSpeed() << std::endl;
    std::cout << "voltage: " << statusDutyCycleGet() << " current: " << statusCurrentGet() << " position: " << statusPositionGet() << " speed: " << statusSpeedGet() << std::endl;
  }

} // namespace puma_motor_driver
