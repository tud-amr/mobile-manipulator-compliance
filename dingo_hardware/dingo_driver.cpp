#include "dingo_driver.h"

#include <iostream>
#include "puma_motor_msg/Status.h"

namespace dingo_driver
{
    Actuator::Actuator(std::string name, puma_motor_driver::Driver driver) : driver_(driver)
    {
        state.name = name;
    }

    puma_motor_driver::Driver *Actuator::get_driver()
    {
        return &driver_;
    }

    DriverManager::DriverManager(std::string canbus_name)
    {
        gateway_.reset(new puma_motor_driver::SocketCANGateway(canbus_name));
    }

    void DriverManager::connect_gateway()
    {
        if (!gateway_->isConnected())
        {
            if (!gateway_->connect())
            {
                std::cout << "Error connecting to motor driver gateway." << std::endl;
            }
            else
            {
                std::cout << "Connection to motor driver gateway successful." << std::endl;
            }
            return;
        }
        std::cout << "Already connected to gateway." << std::endl;
    }

    void DriverManager::add_actuator(int can_id, std::string name)
    {
        puma_motor_driver::Driver driver(*gateway_, can_id, name);
        Actuator actuator(name, driver);
        actuators_.insert({name, actuator});
    }

    void DriverManager::initialize_encoders()
    {
        for (auto &[name, actuator] : actuators_)
        {
            actuator.get_driver()->setPosEncoderRef();
            actuator.get_driver()->setSpdEncoderRef();
        }
    }

    std::vector<State> DriverManager::get_states()
    {
        std::vector<State> states;
        for (auto &[name, actuator] : actuators_)
        {
            State state;
            puma_motor_driver::Driver *driver = actuator.get_driver();
            driver->clearMsgCache();
            driver->requestFeedbackPosition();
            driver->requestFeedbackSpeed();
            driver->requestFeedbackVoltOut();
            driver->requestFeedbackCurrent();
            while (!driver->lastPositionReceived() || !driver->lastSpeedReceived() || !driver->lastOutVoltageReceived() || !driver->lastCurrentReceived())
            {
                // std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            state.name = driver->deviceName();
            state.position = actuator.get_driver()->lastPosition();
            state.speed = actuator.get_driver()->lastSpeed();
            state.voltage = actuator.get_driver()->lastOutVoltage();
            state.current = actuator.get_driver()->lastCurrent();
            states.push_back(state);
        }
        return states;
    }

    double DriverManager::get_gain(std::string name, std::string mode, std::string gain)
    {
        return get_actuator_(name)->get_driver()->getGain(mode, gain);
    }

    void DriverManager::set_mode(std::string name, std::string mode)
    {
        uint8_t mode_id;

        if (mode == "Vol")
        {
            mode_id = puma_motor_msgs::Status::MODE_VOLTAGE;
        }
        else if (mode == "Cur")
        {
            mode_id = puma_motor_msgs::Status::MODE_CURRENT;
        }
        else if (mode == "Pos")
        {
            mode_id = puma_motor_msgs::Status::MODE_POSITION;
        }
        else
        {
            std::cout << "Mode not provided, please select 'Vol', 'Cur' or 'Pos' as mode." << std::endl;
            return;
        }

        std::cout << "Trying to set '" << name << "' to mode " << unsigned(mode_id) << "..." << std::endl;
        try
        {
            get_actuator_(name)->get_driver()->setMode(mode_id);
        }
        catch (std::invalid_argument &exception)
        {
            std::cout << exception.what() << std::endl;
        }
    }

    void DriverManager::set_gain(std::string name, std::string mode, std::string gain, double value)
    {
        std::cout << "Trying to set gain '" << mode + gain << "' of '" << name << "' to " << value << "..." << std::endl;
        try
        {
            get_actuator_(name)->get_driver()->setGain(mode, gain, value);
        }
        catch (std::invalid_argument &exception)
        {
            std::cout << exception.what() << std::endl;
        }
    }

    void DriverManager::command(std::string name, std::string mode, double value)
    {
        puma_motor_driver::Driver *driver;
        try
        {
            driver = get_actuator_(name)->get_driver();
        }
        catch (std::invalid_argument &exception)
        {
            std::cout << exception.what() << std::endl;
            return;
        }

        if (mode == "Vol")
        {
            driver->commandDutyCycle(value);
        }
        else if (mode == "Cur")
        {
            driver->commandCurrent(value);
        }
        else if (mode == "Spd")
        {
            driver->commandSpeed(value);
        }
    }

    void DriverManager::canread()
    {

        for (auto &[name, actuator] : actuators_)
        {
            actuator.get_driver()->canRead();
        }
    }

    Actuator *DriverManager::get_actuator_(std::string name)
    {
        auto it = actuators_.find(name);
        if (it == actuators_.end())
        {
            throw std::invalid_argument("ERROR: Actuator '" + name + "' was not found.");
        }
        else
        {
            return &it->second;
        }
    }

}
