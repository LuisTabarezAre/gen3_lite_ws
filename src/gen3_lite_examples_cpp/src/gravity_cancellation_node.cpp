#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "components_cpp/dynamic_model.hpp"

#include <unordered_map>
#include <mutex>

class GravityCancellationNode : public rclcpp::Node
{
public:
    GravityCancellationNode()
    : Node("gravity_cancellation_node",
           rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
      js_received_(false)
    {
        publisher_effort_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/r1/effort_controller/commands", 10);

        publisher_gravity_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/r1/gravity_effort", 10);

        subscriber_js_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/r1/joint_states",
            rclcpp::SensorDataQoS(),
            std::bind(&GravityCancellationNode::jointStateCallback, this, std::placeholders::_1)
        );

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&GravityCancellationNode::computeGravity, this)
        );

        RCLCPP_INFO(this->get_logger(), "Gravity Cancellation Node started!");
    }

private:
    // ============================================================
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(js_mutex_);
        joint_state_ = *msg;
        js_received_ = true;
    }

    // ============================================================
    void computeGravity()
    {
        if (!js_received_) return;

        sensor_msgs::msg::JointState js;
        {
            std::lock_guard<std::mutex> lock(js_mutex_);
            js = joint_state_;
        }

        // Map joint name → position
        std::unordered_map<std::string, double> q_map;
        for (size_t i = 0; i < js.name.size(); ++i)
            q_map[js.name[i]] = js.position[i];

        // Joints WITH PREFIX
        const std::vector<std::string> joints = {
            "r1_joint_1",
            "r1_joint_2",
            "r1_joint_3",
            "r1_joint_4",
            "r1_joint_5",
            "r1_joint_6"
        };

        // Safety check
        for (const auto &j : joints)
        {
            if (!q_map.count(j))
            {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 2000,
                    "Joint %s not found in joint_states", j.c_str());
                return;
            }
        }

        // Build q vector (order matters!)
        std::vector<double> q(6);
        for (size_t i = 0; i < 6; ++i)
            q[i] = q_map[joints[i]];

        // Gravity computation
        std::vector<double> G = calc_gravity(q);

        // Publish
        std_msgs::msg::Float64MultiArray msg;
        msg.data = G;

        publisher_effort_->publish(msg);
        publisher_gravity_->publish(msg);
    }

    // ============================================================
    std::mutex js_mutex_;
    sensor_msgs::msg::JointState joint_state_;
    bool js_received_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_effort_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_gravity_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_js_;
    rclcpp::TimerBase::SharedPtr timer_;
};

// ================================================================
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GravityCancellationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
