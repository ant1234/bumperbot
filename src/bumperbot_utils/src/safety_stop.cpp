#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"

using std::placeholders::_1;

enum State 
{
    FREE = 0,
    WARNING, 
    DANGER
};

class SafetyStop : public rclcpp::Node 
{
    public:
        SafetyStop() : Node("safety_stop_node")
        {
            declare_parameter<double>("danger_distance", 0.2);
            declare_parameter<std::string>("scan_topic", "scan");
            declare_parameter<std::string>("safety_stop_topic", "safety_stop");

            danger_distance_ = get_parameter("danger_distance").as_double();

            std::string scan_topic = get_parameter("scan_topic").as_string();
            std::string safety_stop_topic  = get_parameter("safety_stop_topic").as_string();

            state_ = State::FREE;

            laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(scan_topic, 10, std::bind(&SafetyStop::laserCallback, this, _1));

            safety_stop_pub_ = create_publisher<std_msgs::msg::Bool>(safety_stop_topic, 10);
        }

    private:
        double danger_distance_;
        State state_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr safety_stop_pub_;

        void laserCallback(const sensor_msgs::msg::LaserScan & msg)
        {
            state_ = State::FREE;

            for (const auto & range : msg.ranges)
            {
                if(range <= danger_distance_)
                {
                    state_ = State::DANGER;
                    break;
                }
            }

            std_msgs::msg::Bool is_safety_stop;
            if(state_ == State::DANGER) 
            {
                is_safety_stop.data = true;
            } else if(state_ == State::FREE)
            {
                is_safety_stop.data = false;
            }

            safety_stop_pub_->publish(is_safety_stop);
        }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafetyStop>());
    rclcpp::shutdown();

    return 0;
}