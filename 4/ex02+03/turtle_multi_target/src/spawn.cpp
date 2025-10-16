#include "turtlesim/srv/spawn.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

using turtlesim::srv::Spawn;

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node =
        rclcpp::Node::make_shared("turtlesim_spawn_client");

    auto client = node->create_client<Spawn>("spawn");

    auto req = std::make_shared<Spawn::Request>();

    req->name = node->declare_parameter<std::string>("name");
    req->x = node->declare_parameter<float>("x");
    req->y = node->declare_parameter<float>("y");
    req->theta = node->declare_parameter<float>("theta");

    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                         "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "service not available, waiting again...");
    }

    auto resp = client->async_send_request(req);

    if (rclcpp::spin_until_future_complete(node, resp) !=
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                     "Failed to call spawn service");
    }

    rclcpp::shutdown();
    return 0;
}