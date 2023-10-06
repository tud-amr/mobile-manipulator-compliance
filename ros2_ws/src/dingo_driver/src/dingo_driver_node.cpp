#include <chrono>
#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "dingo_driver_msg/msg/feedback.hpp"
#include "dingo_driver_msg/msg/command.hpp"
#include "dingo_driver_msg/srv/set_gain.hpp"
#include "dingo_driver.h"

class DingoDriverNode : public rclcpp::Node
{
public:
    DingoDriverNode()
        : Node("dingo_driver_node"), driver_manager_("vcan0")
    {
        publisher_ = this->create_publisher<dingo_driver_msg::msg::Feedback>("/dingo_driver/feedback", 10);
        subscription_ = this->create_subscription<dingo_driver_msg::msg::Command>("/dingo_driver/command", 10, std::bind(&DingoDriverNode::set_command, this, std::placeholders::_1));
        service_ = this->create_service<dingo_driver_msg::srv::SetGain>("set_gain", std::bind(&DingoDriverNode::set_gain, this, std::placeholders::_1, std::placeholders::_2));
    }

    void initialize_drivers()
    {
        driver_manager_.connect_gateway();
        driver_manager_.add_actuator(canbus_id_, acutator_);
        driver_manager_.set_mode(acutator_, "Cur");
        driver_manager_.initialize_encoders();
    }

    void set_command(const dingo_driver_msg::msg::Command command)
    {
        command_ = command.value;
    }

    void set_gain(const std::shared_ptr<dingo_driver_msg::srv::SetGain::Request> request,
                  std::shared_ptr<dingo_driver_msg::srv::SetGain::Response> response)
    {
        change_gain_ = true;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Driver: Set gain...");
        driver_manager_.set_gain(acutator_, "Cur", request->gain, request->value);
        response->set__success(true);
        change_gain_ = false;
    }

    void canread_loop()
    {
        while (true)
        {
            driver_manager_.canread();
        }
    }

    void command_loop()
    {
        while (true)
        {
            if (change_gain_)
                continue;
            driver_manager_.command(acutator_, "Cur", command_);
            // command_ = 0;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    void feedback_loop()
    {
        initialize_drivers();
        dingo_driver::State state;
        dingo_driver_msg::msg::Feedback feedback;
        int x = 0;
        while (true)
        {
            if (change_gain_)
                continue;
            state = driver_manager_.get_states()[0];
            feedback.set__x(x);
            feedback.set__position(state.position);
            feedback.set__speed(state.speed);
            feedback.set__voltage(state.voltage);
            feedback.set__current(state.current);
            publisher_->publish(feedback);
            x += 1;
        }
    }

private:
    rclcpp::Publisher<dingo_driver_msg::msg::Feedback>::SharedPtr publisher_;
    rclcpp::Subscription<dingo_driver_msg::msg::Command>::SharedPtr subscription_;
    rclcpp::Service<dingo_driver_msg::srv::SetGain>::SharedPtr service_;
    dingo_driver::DriverManager driver_manager_;
    float command_ = 0;
    bool change_gain_ = false;
    std::string acutator_ = "rear_right_wheel";
    int canbus_id_ = 5;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    DingoDriverNode dingo_driver_node;
    std::thread canread_thread(&DingoDriverNode::canread_loop, &dingo_driver_node);
    std::thread feedback_thread(&DingoDriverNode::feedback_loop, &dingo_driver_node);
    std::thread command_thread(&DingoDriverNode::command_loop, &dingo_driver_node);
    rclcpp::Node::SharedPtr pointer(&dingo_driver_node);
    rclcpp::spin(pointer);
    std::terminate();
    rclcpp::shutdown();
    return 0;
}