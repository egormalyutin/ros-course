#include "rclcpp/rclcpp.hpp"

#include "std_srvs/srv/trigger.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using std_srvs::srv::Trigger;

using namespace std::chrono_literals;

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node =
        rclcpp::Node::make_shared("next_target_client");

    auto logger = rclcpp::get_logger("rclcpp");

    auto client = node->create_client<Trigger>("next_target");

    auto request = std::make_shared<Trigger::Request>();

    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(logger,
                         "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(logger, "service not available, waiting again...");
    }

    RCLCPP_INFO(logger, "ready");

    std::string line;
    while (true) {
        std::getline(std::cin, line);
        if (std::cin.fail() || std::cin.eof()) {
            break;
        }

        if (line != "n") {
            RCLCPP_INFO(logger, "unknown command %s", line.c_str());
            continue;
        }

        auto result = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node, result) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            auto r = result.get();
            RCLCPP_INFO(logger, "Success: %d, message: %s", r->success,
                        r->message.c_str());
        } else {
            RCLCPP_ERROR(logger, "Failed to call service next_target");
        }
    }

    rclcpp::shutdown();
    return 0;
}
