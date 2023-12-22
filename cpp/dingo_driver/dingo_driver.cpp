#include "dingo_driver.h"

#include <iostream>
#include "puma_motor_msg/Status.h"

namespace dingo_driver
{
    Actuator::Actuator(std::string name, puma_motor_driver::Driver driver, bool flip) : driver_(driver), flip_value(flip ? -1 : 1)
    {
        state.name = name;
    }

    puma_motor_driver::Driver *Actuator::get_driver()
    {
        return &driver_;
    }

    DriverManager::DriverManager(std::string canbus_name, std::string mode) : mode_(mode)
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
        }
        else
        {
            std::cout << "Already connected to gateway." << std::endl;
        }
    }

    void DriverManager::canread_loop()
    {
        while (true)
        {
            gateway_->canRead();
        }
    }

    void DriverManager::start_canread_loop()
    {
        canread_thread_ = std::thread(&dingo_driver::DriverManager::canread_loop, this);
    }

    void DriverManager::add_actuator(int can_id, std::string name, bool flip)
    {
        puma_motor_driver::Driver driver(*gateway_, can_id, name);
        Actuator actuator(name, driver, flip);
        actuators_.insert({name, actuator});
        gateway_->addDriver(get_actuator_(name)->get_driver());
    }

    void DriverManager::set_mode(std::string name)
    {
        uint8_t mode_id;

        if (mode_ == "Vol")
        {
            mode_id = puma_motor_msgs::Status::MODE_VOLTAGE;
        }
        else if (mode_ == "Vel")
        {
            mode_id = puma_motor_msgs::Status::MODE_SPEED;
        }
        else
        {
            std::cout << "Mode not provided, please select 'Vol' or 'Vel' as mode." << std::endl;
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

    void DriverManager::initialize_encoders()
    {
        for (auto &[name, actuator] : actuators_)
        {
            actuator.get_driver()->setPosEncoderRef();
            actuator.get_driver()->setSpdEncoderRef();
        }
    }

    void DriverManager::update_loop()
    {
        add_actuators();
        while (true)
        {
            update_state();
        }
    }

    void DriverManager::start_update_loop()
    {
        update_thread_ = std::thread(&dingo_driver::DriverManager::update_loop, this);
    }

    void DriverManager::add_actuators()
    {
        add_actuator(2, "front_left", false);
        add_actuator(3, "front_right", true);
        add_actuator(4, "rear_left", false);
        add_actuator(5, "rear_right", true);
        set_mode("front_left");
        set_mode("front_right");
        set_mode("rear_left");
        set_mode("rear_right");
        initialize_encoders();
    }

    void DriverManager::update_state()
    {
        std::vector<float> pos_vector;
        std::vector<float> tor_vector;
        int n = 0;
        for (auto &[name, actuator] : actuators_)
        {
            puma_motor_driver::Driver *driver = actuator.get_driver();
            driver->clearMsgCache();
            driver->requestFeedbackPosition();
            driver->requestFeedbackVoltOut();
            driver->requestFeedbackCurrent();
            std::chrono::time_point start = std::chrono::steady_clock::now();
            while (!driver->lastPositionReceived() || !driver->lastSpeedReceived() || !driver->lastOutVoltageReceived() || !driver->lastCurrentReceived())
            {
                if (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(50))
                {
                    continue;
                    std::cout << "Resend request." << std::endl;
                }
                if (!driver->lastPositionReceived())
                    driver->requestFeedbackPosition();
                if (!driver->lastOutVoltageReceived())
                    driver->requestFeedbackVoltOut();
                if (!driver->lastCurrentReceived())
                    driver->requestFeedbackCurrent();
                start = std::chrono::steady_clock::now();
            }
            float pos = driver->lastPosition() * actuator.flip_value;
            float vol = driver->lastOutVoltage() * actuator.flip_value;
            float cur = driver->lastCurrent();
            pos_vector.push_back(pos);
            tor_vector.push_back(vol * cur);

            if (mode_ == "Vol")
                driver->commandDutyCycle(command_[n] * actuator.flip_value);
            else if (mode_ == "Vel")
                driver->commandSpeed(command_[n] * actuator.flip_value);
            n += 1;
        }
        pos_ = pos_vector;
        tor_ = tor_vector;
    }

    void DriverManager::set_command(std::vector<float> command)
    {
        command_ = command;
    };

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
