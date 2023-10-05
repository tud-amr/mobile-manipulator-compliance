#include <chrono>
#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "dingo_driver_msg/msg/feedback.hpp"
#include "dingo_driver_msg/srv/set_gain.hpp"
#include "dingo_driver.h"

class DingoDriverNode : public rclcpp::Node
{
public:
    DingoDriverNode()
        : Node("minimal_publisher"), driver_manager_("vcan0")
    {
        publisher_ = this->create_publisher<dingo_driver_msg::msg::Feedback>("/dingo_driver/feedback", 10);
    }

    void initialize_drivers()
    {
        driver_manager_.connect_gateway();
        driver_manager_.add_actuator(5, "rear_right_wheel");
        driver_manager_.initialize_encoders();
    }

    void set_gain(const std::shared_ptr<dingo_driver_msg::srv::SetGain::Request> request,
                  std::shared_ptr<dingo_driver_msg::srv::SetGain::Response> response)
    {
        response->set__success(true);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service called.");
    }

    void canread_loop()
    {
        while (true)
        {
            driver_manager_.canread();
        }
    }

    void publish_loop()
    {
        initialize_drivers();
        dingo_driver::State state;
        dingo_driver_msg::msg::Feedback feedback;
        int x = 0;
        while (true)
        {
            state = driver_manager_.get_states()[0];
            feedback.set__x(x);
            feedback.set__y_feedback(state.current);
            publisher_->publish(feedback);
            x += 1;
        }
    }

private:
    rclcpp::Publisher<dingo_driver_msg::msg::Feedback>::SharedPtr publisher_;
    dingo_driver::DriverManager driver_manager_;
    rclcpp::Service<dingo_driver_msg::srv::SetGain>::SharedPtr service_ = this->create_service<dingo_driver_msg::srv::SetGain>("set_gain", std::bind(&DingoDriverNode::set_gain, this, std::placeholders::_1, std::placeholders::_2));
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    DingoDriverNode dingo_driver_node;
    std::thread canread_thread(&DingoDriverNode::canread_loop, &dingo_driver_node);
    std::thread thread(&DingoDriverNode::publish_loop, &dingo_driver_node);
    rclcpp::Node::SharedPtr pointer(&dingo_driver_node);
    rclcpp::spin(pointer);
    std::terminate();
    rclcpp::shutdown();
    return 0;
}