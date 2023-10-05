#ifndef STATUS_H
#define STATUS_H

#include <string>

namespace puma_motor_msgs
{
    class Status
    {
    public:
        // Number on the bus (CAN ID).
        uint8_t device_number;

        // Name of joint controlled, or other identifier.
        std::string device_name;

        // Input terminal voltage (volts).
        _Float32 bus_voltage;

        // Internal driver temperature (degC).
        _Float32 temperature;

        // Voltage as output to the motor (volts).
        _Float32 output_voltage;

        // Value of the auxiliary ADC (volts).
        _Float32 analog_input;

        // Available control modes, not all of which are broken out to
        // this ROS driver.
        static const uint8_t MODE_VOLTAGE = 0;
        static const uint8_t MODE_CURRENT = 1;
        static const uint8_t MODE_SPEED = 2;
        static const uint8_t MODE_POSITION = 3;
        static const uint8_t MODE_VCOMP = 4;
        uint8_t mode;

        // Fault states which could cause the driver to be immobilized.
        static const uint8_t FAULT_CURRENT = 1;
        static const uint8_t FAULT_TEMPERATURE = 2;
        static const uint8_t FAULT_BUS_VOLTAGE = 4;
        static const uint8_t FAULT_BRIDGE_DRIVER = 8;
        uint8_t fault;
    };
}

#endif