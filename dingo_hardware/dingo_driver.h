#ifndef DINGO_DRIVER
#define DINGO_DRIVER

#include <string>
#include <vector>
#include <map>
#include "puma_motor_driver/socketcan_gateway.h"
#include "puma_motor_driver/driver.h"

namespace dingo_driver
{
    /**
     * @brief Contains the state elements of an actuator.
     *
     */
    struct State
    {
        std::string name;
        double position;
        double speed;
        double voltage;
        double current;
    };

    /**
     * @brief Contains the driver and state object of an actuator.
     *
     */
    struct Actuator
    {
    public:
        Actuator(std::string name, puma_motor_driver::Driver driver);
        puma_motor_driver::Driver *get_driver();
        dingo_driver::State state;

    private:
        puma_motor_driver::Driver driver_;
    };

    /**
     * @brief Manages the actuators of the Dingo.
     *
     */
    class DriverManager
    {
    public:
        /**
         * @brief Construct a new Driver Manager object
         *
         * @param canbus_name
         */
        DriverManager(std::string canbus_name);

        /**
         * @brief Connect to the canbus gateway.
         *
         */
        void connect_gateway();

        /**
         * @brief Add an actuator to the driver manager.
         *
         * @param can_id "The can id of the actuator."
         * @param name "The name of the actuator."
         */
        void add_actuator(int can_id, std::string name);

        /**
         * @brief Initialize the encoders for all actuators.
         *
         */
        void initialize_encoders();

        /**
         * @brief Get the states of all actuators.
         *
         * @return std::vector<State>
         */
        std::vector<State> get_states();

        /**
         * @brief Get the gain object
         *
         * @param name "The name of the actuator."
         * @param mode "Cur", "Spd"
         * @param gain "P", "I", "D"
         * @return double
         */
        double get_gain(std::string name, std::string mode, std::string gain);

        /**
         * @brief Set the mode of an actuator.
         *
         * @param name "The name of the actuator."
         * @param mode "Vol", "Cur", "Spd"
         */
        void set_mode(std::string name, std::string mode);

        /**
         * @brief Set the gain of an actuator.
         *
         * @param name "The name of the actuator."
         * @param mode "Cur", "Spd"
         * @param gain "P", "I", "D"
         * @param value "Gain value."
         */
        void set_gain(std::string name, std::string mode, std::string gain, double value);

        /**
         * @brief Send a command.
         *
         * @param name "Name of the actuator."
         * @param mode "Vol", "Cur", "Spd"
         * @param value "Vol: between -1 and 1"
         */
        void command(std::string name, std::string mode, double value);

        /**
         * @brief Reads the canbus for all actuators.
         *
         */
        void canread();

    private:
        std::unique_ptr<puma_motor_driver::Gateway> gateway_;
        std::map<std::string, Actuator> actuators_;
        bool canread_loop_;

        Actuator *get_actuator_(std::string name);
    };
}

#endif // DINGO_DRIVER_