#ifndef TEST_NODE_H
#define TEST_NODE_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node{
    public:
        MinimalPublisher();

    private:
        void timer_callback();
};

int main(int argc, char * argv[]);
