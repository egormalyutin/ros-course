#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <tf2_ros/buffer.hpp>

#include "rclcpp/rclcpp.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "std_msgs/msg/string.hpp"
#include "turtlesim/msg/pose.hpp"

#include "turtle_multi_target_interfaces/msg/target_status.hpp"

#include "std_srvs/srv/trigger.hpp"

#include "./throttler.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;
using geometry_msgs::msg::Twist;

using geometry_msgs::msg::TransformStamped;
using std_srvs::srv::Trigger;
using turtle_multi_target_interfaces::msg::TargetStatus;

#define PI 3.1415926

struct TurtleController : public rclcpp::Node {
    TurtleController() : Node("turtle_controller") {
        turtle_frame = declare_parameter<std::string>("turtle_frame");
        target_frames =
            declare_parameter<std::vector<std::string>>("target_frames");
        switch_threshold = declare_parameter<float>("switch_threshold", 1.);
        delay = declare_parameter<float>("delay", 0.);

        assert(target_frames.size() > 0);

        tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        timer =
            create_wall_timer(5ms, std::bind(&TurtleController::update, this));

        cmd_vel_pub = create_publisher<Twist>("cmd_vel", 10);

        status_pub = create_publisher<TargetStatus>("current_target", 10);

        trigger_target_switch = create_service<Trigger>(
            "next_target",
            std::bind(&TurtleController::handle_target_switch, this, _1, _2));

        log_throttler = Throttler(*this, 1s);
    }

  private:
    void update() {
        auto now = get_clock()->now();
        auto &target_frame = target_frames[target_i];

        TransformStamped p;
        try {
            p = lookup_target_tf(target_frame);
        } catch (const tf2::TransformException &ex) {
            if (log_throttler) {
                RCLCPP_WARN(get_logger(), "Could not transform %s to %s: %s",
                            turtle_frame.c_str(), target_frame.c_str(),
                            ex.what());
            }
            return;
        }

        auto tr = p.transform.translation;

        float angle = std::atan2(tr.y, tr.x);
        float dist = std::sqrt(tr.x * tr.x + tr.y * tr.y);

        TargetStatus s;
        s.target_name = target_frame;
        s.distance_to_target = dist;
        s.target_x = tr.x;
        s.target_y = tr.y;
        status_pub->publish(s);

        if (dist > switch_threshold && std::abs(angle) > 0.3) {
            cmd_vel(0, angle * 4.);
        } else if (dist > switch_threshold) {
            cmd_vel(std::min(dist, 1.f) * 2., angle * 4.);
        } else {
            cmd_vel(0, 0);
            if (target_frames.size() > 1) {
                RCLCPP_INFO(get_logger(), "reached %s", target_frame.c_str());
                next_target();
            }
        }
    }

    TransformStamped lookup_target_tf(const std::string &frame) {
        auto duration = std::chrono::duration<float>(delay);
        auto when = this->get_clock()->now() - rclcpp::Duration(duration);
        return tf_buffer->lookupTransform(turtle_frame, now(), frame, when,
                                          "world", 50ms);
    }

    void next_target() {
        target_i = (target_i + 1) % target_frames.size();
    }

    void cmd_vel(float linear, float angular) {
        Twist t;
        t.linear.x = linear;
        t.angular.z = angular;
        cmd_vel_pub->publish(t);
    }

    void handle_target_switch(const std::shared_ptr<Trigger::Request>,
                              std::shared_ptr<Trigger::Response> resp) {
        next_target();
        resp->success = true;

        std::stringstream ss;
        ss << "switched to " << target_frames[target_i];
        resp->message = ss.str();
    }

    std::string turtle_frame;
    std::vector<std::string> target_frames;
    float switch_threshold;
    float delay;

    size_t target_i = 0;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    rclcpp::TimerBase::SharedPtr timer{nullptr};

    rclcpp::Publisher<Twist>::SharedPtr cmd_vel_pub;
    rclcpp::Publisher<TargetStatus>::SharedPtr status_pub;

    rclcpp::Service<Trigger>::SharedPtr trigger_target_switch;

    Throttler log_throttler;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleController>());
    rclcpp::shutdown();
    return 0;
}