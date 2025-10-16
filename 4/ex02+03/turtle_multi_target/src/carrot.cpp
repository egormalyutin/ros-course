#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "./throttler.hpp"

using namespace std::chrono_literals;

struct CarrotNode : public rclcpp::Node {
    CarrotNode() : Node("carrot") {
        frame = declare_parameter<std::string>("frame");
        carrot_frame = declare_parameter<std::string>("carrot");

        radius = declare_parameter<double>("radius");
        speed = declare_parameter<double>("speed");

        tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        timer = create_wall_timer(5ms, std::bind(&CarrotNode::update, this));

        log_throttler = Throttler(*this, 1s);
    }

  private:
    void update() {
        auto now = get_clock()->now();

        geometry_msgs::msg::TransformStamped p;
        try {
            p = tf_buffer->lookupTransform("world", frame, now, 1s);
        } catch (const tf2::TransformException &ex) {
            if (log_throttler) {
                RCLCPP_WARN(get_logger(), "Could not transform %s to world: %s",
                            frame.c_str(), ex.what());
            }
            return;
        }

        double t = (double)now.nanoseconds() * 1e-9;

        double x = std::cos(t * speed) * radius;
        double y = std::sin(t * speed) * radius;

        geometry_msgs::msg::TransformStamped c;

        c.header.stamp = p.header.stamp;
        c.header.frame_id = "world";
        c.child_frame_id = carrot_frame;

        auto tr = p.transform.translation;

        c.transform.translation.x = tr.x + x;
        c.transform.translation.y = tr.y + y;
        c.transform.translation.z = 0;

        c.transform.rotation.x = 0;
        c.transform.rotation.y = 0;
        c.transform.rotation.z = 0;
        c.transform.rotation.w = 1;

        tf_broadcaster->sendTransform(c);
    }

    std::string frame;
    std::string carrot_frame;
    double radius, speed;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    rclcpp::TimerBase::SharedPtr timer;

    Throttler log_throttler;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CarrotNode>());
    rclcpp::shutdown();
    return 0;
}