#include <chrono>
#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "dingo_driver_msg/msg/din_fdbk.hpp"
#include "dingo_driver_msg/msg/din_cmd.hpp"
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
        publisher_ = this->create_publisher<dingo_driver_msg::msg::DinFdbk>("/dingo/fdbk", 10);
        subscription_ = this->create_subscription<dingo_driver_msg::msg::DinCmd>("/dingo/cmd", 10, std::bind(&DingoDriverNode::set_command, this, std::placeholders::_1));
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

    void set_command(const dingo_driver_msg::msg::DinCmd command)
    {
        int n = 0;
        for (auto &wheel : wheels_)
        {
            wheel.command = command.wheel_command[n];
            driver_manager_.command(wheel.name, "Vol", wheel.command);
            n++;
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
        while (true)
        {
            dingo_driver_msg::msg::DinFdbk feedback;
            std::vector<dingo_driver::State> states = driver_manager_.get_states();
            for (auto &state : states)
            {
                feedback.wheel_pos.push_back(state.position);
                feedback.wheel_vel.push_back(state.speed);
                feedback.wheel_tor.push_back(state.current * state.voltage);
            }
            publisher_->publish(feedback);
        }
    }

private:
    rclcpp::Publisher<dingo_driver_msg::msg::DinFdbk>::SharedPtr publisher_;
    rclcpp::Subscription<dingo_driver_msg::msg::DinCmd>::SharedPtr subscription_;
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