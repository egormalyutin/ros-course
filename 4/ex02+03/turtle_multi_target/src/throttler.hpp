#ifndef RATE_LIMITER_HPP
#define RATE_LIMITER_HPP

#include "rclcpp/rclcpp.hpp"

#include <chrono>

struct Throttler {
    Throttler() : clock(nullptr), duration(0, 0) {
    }

    Throttler(rclcpp::Clock::SharedPtr &clock, rclcpp::Duration duration)
        : clock(clock), duration(duration) {
    }

    Throttler(rclcpp::Node &node, rclcpp::Duration duration)
        : clock(node.get_clock()), duration(duration) {
    }

    operator bool() {
        auto now = clock->now();

        if (first) {
            last_trigger = now;
            first = false;
            return true;
        }

        if (now - last_trigger > duration) {
            last_trigger = now;
            return true;
        } else {
            return false;
        }
    }

  private:
    rclcpp::Clock::SharedPtr clock;
    rclcpp::Duration duration;

    rclcpp::Time last_trigger;
    bool first = true;
};

#endif