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
        Actuator(std::string name, puma_motor_driver::Driver driver, bool flip);
        puma_motor_driver::Driver *get_driver();
        dingo_driver::State state;
        int flip_value;

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
         * @param flip "Wether the actuator is flipped."
         */
        void add_actuator(int can_id, std::string name, bool flip);

        /**
         * @brief Initialize the encoders for all actuators.
         *
         */
        void initialize_encoders();

        /**
         * @brief Set the mode of an actuator.
         *
         * @param name "The name of the actuator."
         * @param mode "Vol", "Cur", "Spd"
         */
        void set_mode(std::string name, std::string mode);

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
        void canread_loop();

        void start_canread_loop();

        void add_actuators();

        void update_loop();

        void start_update_loop();

        void update_state();

        void set_command(std::vector<float> commands);

        std::vector<float> pos_;
        std::vector<float> tor_;

    private:
        std::unique_ptr<puma_motor_driver::SocketCANGateway> gateway_;
        std::map<std::string, Actuator> actuators_;
        std::thread canread_thread_;
        std::thread update_thread_;

        std::vector<float> command_ = {0.0, 0.0, 0.0, 0.0};

        Actuator *get_actuator_(std::string name);
    };
}

#endif // DINGO_DRIVER_