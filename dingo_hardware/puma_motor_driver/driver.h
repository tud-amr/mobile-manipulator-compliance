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
#ifndef PUMA_MOTOR_DRIVER_DRIVER_H
#define PUMA_MOTOR_DRIVER_DRIVER_H

#include <stdint.h>
#include <string>
#include <vector>
#include <map>

#include "puma_motor_driver/can_proto.h"

namespace puma_motor_driver
{

  class Gateway;
  class Message;

  class Driver
  {
  public:
    Driver(Gateway &gateway, const uint8_t &device_number, const std::string &device_name);

    void processMessage(const Message &received_msg);

    double radPerSecToRpm() const;

    /**
     * Sends messages to the motor controller requesting all missing elements to
     * populate the cache of status data. Returns true if any messages were sent,
     * false if the cache is already complete.
     */
    void requestStatusMessages();

    /**
     * Sends messages to the motor controller requesting all missing elements to
     * populate the cache of feedback data. Returns true if any messages were sent,
     * false if the cache is already complete.
     */
    void requestFeedbackMessages();

    /**
     * Sends a message to the motor controller requesting the instantaneous voltage output to
     * populate the cache of feedback data.
     */
    void requestFeedbackVoltOut();
    /**
     * Sends a message to the motor controller requesting the instantaneous duty cycle to
     * populate the cache of feedback data.
     */
    void requestFeedbackDutyCycle();
    /**
     * Sends a message to the motor controller requesting the instantaneous current consumption to
     * populate the cache of feedback data.
     */
    void requestFeedbackCurrent();
    /**
     * Sends a message to the motor controller requesting the instantaneous angular distance to
     * populate the cache of feedback data.
     */
    void requestFeedbackPosition();
    /**
     * Sends a message to the motor controller requesting the instantaneous angular speed to
     * populate the cache of feedback data.
     */
    void requestFeedbackSpeed();
    /**
     * Sends a message to the motor controller requesting the state of the power flag.
     */
    void requestFeedbackPowerState();
    /**
     * Sends a message to the motor controller requesting the instantaneous set point of the
     * current control mode to populate the cache of feedback data.
     */
    void requestFeedbackSetpoint();
    /**
     * Clear the received flags from the status cache, in preparation for the next
     * request batch to go out.
     */
    void clearMsgCache();
    /**
     * Command the supplied value in open-loop voltage control.
     *
     * @param[in] cmd Value to command, ranging from -1.0 to 1.0, where zero is neutral.
     */
    void commandDutyCycle(const float cmd);
    /**
     * Command the desired speed set-point in close-loop speed control.
     *
     * @param[in] cmd Value to command in rad/s.
     */
    void commandSpeed(const double cmd);
    /**
     * Command the desired position set-point in close-loop postion control.
     *
     * @param[in] cmd Value to command in rad.
     */
    void commandPosition(const double cmd);
    /**
     * Command the desired current set-point in close-loop current control.
     *
     * @param[in] cmd Value to command in ampere.
     */
    void commandCurrent(const double cmd);
    // void currentSet(float cmd);
    // void positionSet(float cmd);
    // void neutralSet();

    /**
     * Set the encoders resolution in counts per rev.
     *
     * @param[in] encoder_cpr Value to set.
     */
    void setEncoderCPR(const uint16_t encoder_cpr);
    /**
     * Set the gear ratio of the motors.
     *
     * @param[in] gear_ratio Value to set.
     */
    void setGearRatio(const float gear_ratio);
    /**
     * Set the control mode of the motor drivers.
     *
     * @param[in] mode Value to set.
     */
    void setMode(const uint8_t mode);

    /**
     * Process the last received fault response.
     *
     * @return state of fault status.
     */
    uint8_t lastFault();
    /**
     * Process the last received power response.
     *
     * @return state of power status.
     */
    uint8_t lastPower();
    /**
     * Checks whether the power response was received.
     *
     * @return receive status of power response.
     */
    bool lastPowerReceived();
    /**
     * Process the last received mode response.
     *
     * @return current mode of motor driver.
     */
    uint8_t lastMode();
    /**
     * Checks whether the mode response was received.
     *
     * @return receive status of motor driver mode.
     */
    bool lastModeReceived();
    /**
     * Process the last received duty cycle response.
     *
     * @return value of the instantaneous duty cycle.
     */
    float lastDutyCycle();
    /**
     * Process the last received bus voltage response.
     *
     * @return value of the instantaneous bus voltage.
     */
    float lastBusVoltage();
    /**
     * Process the last received current response.
     *
     * @return value of the instantaneous current.
     */
    float lastCurrent();
    /**
     * Checks whether the current response was received.
     *
     * @return receive status of current response.
     */
    bool lastCurrentReceived();
    /**
     * Process the last received out voltage response.
     *
     * @return value of the instantaneous out voltage.
     */
    float lastOutVoltage();
    /**
     * Checks whether the voltage response was received.
     *
     * @return receive status of voltage response.
     */
    bool lastOutVoltageReceived();
    /**
     * Process the last received temperature response.
     *
     * @return value of the instantaneous temperature.
     */
    float lastTemperature();
    /**
     * Process the last received analog_input response.
     *
     * @return value of the instantaneous analog_input.
     */
    float lastAnalogInput();
    /**
     * Process the last received travel response.
     *
     * @return value of the instantaneous angular position.
     */
    double lastPosition();
    /**
     * Checks whether the position response was received.
     *
     * @return receive status of position response.
     */
    bool lastPositionReceived();
    /**
     * Process the last received speed response.
     *
     * @return value of the instantaneous angular speed.
     */
    double lastSpeed();
    /**
     * Checks whether the speed response was received.
     *
     * @return receive status of speed response.
     */
    bool lastSpeedReceived();

    /**
     * Process the last received position encoder reference response
     *
     * @return value of the reference response.
     */
    uint8_t posEncoderRef();
    /**
     * Checks whether the encoder response was received.
     *
     * @return receive status of encoder response.
     */
    bool posEncoderRefReceived();
    /**
     * Process the last received speed encoder reference response
     *
     * @return value of the reference response.
     */
    uint8_t spdEncoderRef();
    /**
     * Checks whether the encoder response was received.
     *
     * @return receive status of encoder response.
     */
    bool spdEncoderRefReceived();
    /**
     * Process the last received encoder counts response
     *
     * @return value of the encoder counts.
     */
    uint16_t encoderCounts();

    /**
     * Process the last received set-point response
     * in voltage open-loop control.
     *
     * @return value of the set-point response.
     */
    float statusDutyCycleGet();
    /**
     * Process the last received set-point response
     * in speed closed-loop control.
     *
     * @return value of the set-point response.
     */
    double statusSpeedGet();
    /**
     * Process the last received set-point response
     * in currrent closed-loop control.
     *
     * @return value of the set-point response.
     */
    float statusCurrentGet();
    /**
     * Process the last received set-point response
     * in position closed-loop control.
     *
     * @return value of the set-point response.
     */
    double statusPositionGet();

    std::string deviceName() const { return device_name_; }

    uint8_t deviceNumber() const { return device_number_; }

    // Only used internally but is used for testing.
    struct Field
    {
      uint8_t data[4];
      bool received;

      float interpretFixed8x8()
      {
        return *(reinterpret_cast<int16_t *>(data)) / static_cast<float>(1 << 8);
      }

      double interpretFixed16x16()
      {
        return *(reinterpret_cast<int32_t *>(data)) / static_cast<double>(1 << 16);
      }
    };

    /** Read the canbus. */
    void canRead();

    /**
     * Get the expected double after converting forth and back to Fixed16x16.
     *
     * @param[in] value the original value.
     * @return the expected value after conversions.
     */
    double expectedDoubleAfter16x16Conversion(double value);
    /**
     * Get the expected double after converting forth and back to Fixed8x8.
     *
     * @param[in] value the original value.
     * @return the expected value after conversions.
     */
    double expectedDoubleAfter8x8Conversion(double value);
    /** Set the posEncoderref. */
    void setPosEncoderRef();
    /** Set the spdEncoderref. */
    void setSpdEncoderRef();
    /**
     * Get the current gain for the given mode/gain combination.
     *
     * @param[in] mode "Cur", "Pos", "Spd".
     * @param[in] gain "P", "I", "D".
     * @param[in] clear_first wheter to clear the message cache first.
     * @return value of the gain.
     */
    double getGain(std::string mode, std::string gain, bool clear_first = false);
    /**
     * Set the current gain for the given mode/gain combination.
     *
     * @param[in] mode "Cur", "Pos", "Spd".
     * @param[in] gain "P", "I", "D".
     * @param[in] value the value to set.
     */
    void setGain(std::string mode, std::string gain, double value);
    /** Print PID values. */
    void printPIDs();
    /** Print driver feedback. */
    void printFeedback();

  private:
    Gateway &gateway_;
    uint8_t device_number_;
    std::string device_name_;

    uint16_t encoder_cpr_;
    float gear_ratio_;
    std::map<std::string, int> pid_apis_;

    /**
     * Helpers to generate data for CAN messages.
     */
    void sendUint8(const uint32_t id, const uint8_t value);
    void sendUint16(const uint32_t id, const uint16_t value);
    void sendFixed8x8(const uint32_t id, const float value);
    void sendFixed16x16(const uint32_t id, const double value);

    /**
     * Comparing the raw bytes of the 16x16 fixed-point numbers
     * to avoid comparing the floating point values.
     *
     * @return boolean if received is equal to expected.
     */
    bool verifyRaw16x16(const uint8_t *received, const double expected);

    /**
     * Comparing the raw bytes of the 8x8 fixed-point numbers
     * to avoid comparing the floating point values.
     *
     * @return boolean if received is equal to expected.
     */
    bool verifyRaw8x8(const uint8_t *received, const float expected);

    Field voltage_fields_[4];
    Field spd_fields_[7];
    Field vcomp_fields_[5];
    Field pos_fields_[7];
    Field ictrl_fields_[6];
    Field status_fields_[15];
    Field cfg_fields_[15];

    Field *voltageFieldForMessage(const Message &msg);
    Field *spdFieldForMessage(const Message &msg);
    Field *vcompFieldForMessage(const Message &msg);
    Field *posFieldForMessage(const Message &msg);
    Field *ictrlFieldForMessage(const Message &msg);
    Field *statusFieldForMessage(const Message &msg);
    Field *cfgFieldForMessage(const Message &msg);
  };

} // namespace puma_motor_driver

#endif // PUMA_MOTOR_DRIVER_DRIVER_H
