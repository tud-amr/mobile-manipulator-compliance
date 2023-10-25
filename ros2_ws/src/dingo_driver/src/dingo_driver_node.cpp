#include <chrono>
#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "dingo_driver_msg/msg/dingo_feedback.hpp"
#include "dingo_driver_msg/msg/dingo_command.hpp"
#include "dingo_driver_msg/srv/set_gain.hpp"
#include "dingo_driver/dingo_driver.h"

struct Wheel
{
    Wheel(std::string name_, int canbus_id_) : name(name_), canbus_id(canbus_id_), command(0) {}
    std::string name;
    int canbus_id;
    float command;
};

class DingoDriverNode : public rclcpp::Node
{
public:
    DingoDriverNode()
        : Node("dingo_driver_node"), driver_manager_("vcan0")
    {
        publisher_ = this->create_publisher<dingo_driver_msg::msg::DingoFeedback>("/dingo/feedback", 10);
        subscription_ = this->create_subscription<dingo_driver_msg::msg::DingoCommand>("/dingo/command", 10, std::bind(&DingoDriverNode::set_command, this, std::placeholders::_1));
        // service_ = this->create_service<dingo_driver_msg::srv::SetGain>("set_gain", std::bind(&DingoDriverNode::set_gain, this, std::placeholders::_1, std::placeholders::_2));
    }

    void initialize_drivers()
    {
        driver_manager_.connect_gateway();
        for (auto &wheel : wheels_)
        {
            driver_manager_.add_actuator(wheel.canbus_id, wheel.name);
            driver_manager_.set_mode(wheel.name, "Vol");
        }
        driver_manager_.initialize_encoders();
    }

    void set_command(const dingo_driver_msg::msg::DingoCommand command)
    {
        for (auto &wheel : wheels_)
        {
            if (wheel.name == "front_left_wheel")
                wheel.command = command.fl;
            else if (wheel.name == "front_right_wheel")
            {
                wheel.command = command.fr;
            }
            else if (wheel.name == "rear_left_wheel")
            {
                wheel.command = command.rl;
            }
            else if (wheel.name == "rear_right_wheel")
            {
                wheel.command = command.rr;
            }
            driver_manager_.command(wheel.name, "Vol", wheel.command);
        }
    }

    void canread_loop()
    {
        while (true)
        {
            driver_manager_.canread();
        }
    }

    void feedback_loop()
    {
        dingo_driver_msg::msg::DingoFeedback feedback;
        int x = 0;
        while (true)
        {
            std::vector<dingo_driver::State> states = driver_manager_.get_states();
            set_wheel_feedbacks(states, feedback);
            feedback.set__time(x);
            publisher_->publish(feedback);
            x += 1;
        }
    }

    void set_wheel_feedbacks(std::vector<dingo_driver::State> &states, dingo_driver_msg::msg::DingoFeedback &feedback)
    {
        for (auto state : states)
        {
            dingo_driver_msg::msg::WheelFeedback wheel_feedback;
            wheel_feedback.set__position(state.position);
            wheel_feedback.set__speed(state.speed);
            wheel_feedback.set__voltage(state.voltage);
            wheel_feedback.set__current(state.current);

            if (state.name == "front_left_wheel")
                feedback.set__front_left_wheel(wheel_feedback);
            else if (state.name == "front_right_wheel")
            {
                feedback.set__front_right_wheel(wheel_feedback);
            }
            else if (state.name == "rear_left_wheel")
            {
                feedback.set__rear_left_wheel(wheel_feedback);
            }
            else if (state.name == "rear_right_wheel")
            {
                feedback.set__rear_right_wheel(wheel_feedback);
            }
        }
    }

private:
    rclcpp::Publisher<dingo_driver_msg::msg::DingoFeedback>::SharedPtr publisher_;
    rclcpp::Subscription<dingo_driver_msg::msg::DingoCommand>::SharedPtr subscription_;
    rclcpp::Service<dingo_driver_msg::srv::SetGain>::SharedPtr service_;
    dingo_driver::DriverManager driver_manager_;
    std::vector<Wheel> wheels_ = {Wheel("front_left_wheel", 2), Wheel("front_right_wheel", 3), Wheel("rear_left_wheel", 4), Wheel("rear_right_wheel", 5)};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    DingoDriverNode dingo_driver_node;
    std::thread canread_thread(&DingoDriverNode::canread_loop, &dingo_driver_node);
    dingo_driver_node.initialize_drivers();
    std::thread feedback_thread(&DingoDriverNode::feedback_loop, &dingo_driver_node);
    rclcpp::Node::SharedPtr pointer(&dingo_driver_node);
    rclcpp::spin(pointer);
    std::terminate();
    rclcpp::shutdown();
    return 0;
}