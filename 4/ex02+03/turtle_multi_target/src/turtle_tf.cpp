#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include "std_msgs/msg/string.hpp"
#include "turtlesim/msg/pose.hpp"

using namespace std::placeholders;

struct TurtleTfNode : public rclcpp::Node {
    TurtleTfNode() : Node("turtle_tf") {
        frame = declare_parameter<std::string>("frame");

        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        sub = create_subscription<turtlesim::msg::Pose>(
            "pose", 10, std::bind(&TurtleTfNode::handle, this, _1));
    }

  private:
    std::string frame;

    void handle(const std::shared_ptr<turtlesim::msg::Pose> msg) {
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = get_clock()->now();
        t.header.frame_id = "world";
        t.child_frame_id = frame.c_str();

        t.transform.translation.x = msg->x;
        t.transform.translation.y = msg->y;
        t.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, msg->theta);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tf_broadcaster->sendTransform(t);
    }

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleTfNode>());
    rclcpp::shutdown();
    return 0;
}