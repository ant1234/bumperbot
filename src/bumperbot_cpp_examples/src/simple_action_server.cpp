#include "thread"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "bumperbot_msgs/action/fibonacci.hpp"

using namespace std::placeholders;

namespace bumperbot_cpp_examples
{
    class SimpleActionServer : public rclcpp::Node 
    {
        public:
            explicit SimpleActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) 
            : Node("simple_action_server", options)
            {
                action_server_ = rclcpp_action::create_server<bumperbot_msgs::action::Fibonacci>(this, "fibonacci",
                    std::bind(&SimpleActionServer::goalCallback, this, _1, _2), 
                    std::bind(&SimpleActionServer::cancelCallback, this, _1), 
                    std::bind(&SimpleActionServer::acceptedCallback, this, _1)
                );
                RCLCPP_INFO(get_logger(), "Starting the Action server");
            }
        private:
            rclcpp_action::Server<bumperbot_msgs::action::Fibonacci>::SharedPtr action_server_;

            rclcpp_action::GoalResponse goalCallback(const rclcpp_action::GoalUUID & uuid, 
            std::shared_ptr<const bumperbot_msgs::action::Fibonacci::Goal> goal)
            {
                RCLCPP_INFO(get_logger(), "Recieved goal request with order %d", goal->order);
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            }

            void acceptedCallback(shared_ptr<rclcpp_action::ServerGoalHandle<bumperbot_msgs::action::Fibonacci>> goal_handle)
            {
                std::thread{std::bind(&SimpleActionServer::execute, this, _1), goal_handle}.detach();
            }

            void execute(shared_ptr<rclcpp_action::ServerGoalHandle<bumperbot_msgs::action::Fibonacci>> goal_handle)
            {
                RCLCPP_INFO(get_logger(), "Executing goal");
                rclcpp::Rate loop_rate(1);
                const auto goal = goal_handle->get_goal();
                auto feedback = std::make_shared<bumperbot_msgs::action::Fibonacci::Feedback>();
                auto & sequence = feedback->partial_sequence;
                sequence.push_back(0);
                sequence.push_back(0);
                auto result = std::make_shared<bumperbot_msgs::action::Fibonacci::Result>();

                for(int i = 0; (i < goal->order) && rclcpp::ok(); i++)
                { 
                    if(goal_handle->is_cancelling())
                    {
                        result->sequence = sequence;
                        goal_handle->canceled(result);
                        RCLCPP_INFO(get_logger(), "Goal canceled");
                        return;
                    }

                    sequence.push_back(sequence[i] + sequence[i - 1]);
                    goal_handle->publish_feedback(feedback);
                    RCLCPP_INFO(get_logger(), "Publishing feedback");
                    loop_rate.sleep();
                }

                if(rclcpp::ok())
                {
                    result->sequence = sequence;
                    goal_handle->succeed(result);
                    RCLCPP_INFO(get_logger(), "Goal Succeeded");
                }
            }

            rclcpp_action::cancelResponse cancelCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<bumperbot_msgs::action::Fibonacci>> goal_handle)
            {
                RCLCPP_INFO(get_logger(), "Recieved request to cancel the goal");
                return rclcpp_action::cancelResponse::ACCEPT;
            }

    };
}

RCLCPP_COMPONENTS_REGISTER_NODE(bumperbot_cpp_examples::SimpleActionServer)
