#include <functional>
#include <memory>
#include <thread>
#include <chrono>
#include <future>


#include "rclcpp/rclcpp.hpp"
#include "action_template_interfaces/action/count_until.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"


class CountUntilClientNode : public rclcpp::Node
{
public:
    using CountUntil = action_template_interfaces::action::CountUntil;
    using GoalHandleCountUntil = rclcpp_action::ClientGoalHandle<CountUntil>;

    explicit CountUntilClientNode(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
     : Node("count_until_action_client", node_options)
    {
        this->declare_parameter("target_number", 10);
        this->declare_parameter("period", 1.0);

        // creating action client
        this->count_until_client_ = rclcpp_action::create_client<CountUntil>(
            this->get_node_base_interface(),
            this->get_node_graph_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(), 
            "CountUntil");
        RCLCPP_INFO(this->get_logger(), "Action client has been started.");

        // Start the goal sending process after a short delay to allow service discovery
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&CountUntilClientNode::send_goal_from_parameters, this));
    }

    void send_goal_from_parameters()
    {
        // Cancel the timer to prevent multiple calls
        timer_->cancel();
        
        // Get parameters
        long int target_number = this->get_parameter("target_number").as_int();
        float period = this->get_parameter("period").as_double();
        
        RCLCPP_INFO(this->get_logger(), "Sending goal with target_number: %ld, period: %.2f", 
                    target_number, period);
                    
        send_goal(target_number, period);
    }
 
    void send_goal(long int t_number, float period)
    {
        using namespace std::placeholders;  // Add this line for _1, _2 placeholders

        // wait for action server and it not available then exiting the node
        if(!this->count_until_client_->wait_for_action_server(std::chrono::seconds(10)))
        {
            RCLCPP_INFO(this->get_logger(), "After wait action server is not available. Sorry!!!");
            rclcpp::shutdown();
            return;
        }

        // defining goal
        auto goal_msg = CountUntil::Goal();
        goal_msg.target_number = t_number;
        goal_msg.period = period;

        // define callbacks for goal response, feedback and result
        auto send_goal_options = rclcpp_action::Client<CountUntil>::SendGoalOptions();
        // send_goal_options.goal_response_callback = std::bind(&CountUntilClientNode::goal_response_callback, this, _1);
        send_goal_options.goal_response_callback = [this](const GoalHandleCountUntil::SharedPtr & goal_handle)
        {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
        };
        send_goal_options.feedback_callback = std::bind(&CountUntilClientNode::feedback_callback, this, _1, _2);
        send_goal_options.result_callback = std::bind(&CountUntilClientNode::result_callback, this, _1);
        this->count_until_client_->async_send_goal(goal_msg, send_goal_options);

    }

private:
    rclcpp_action::Client<CountUntil>::SharedPtr count_until_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    // void goal_response_callback(std::shared_future<GoalHandleCountUntil::SharedPtr> future)
    // {
    //     auto goal_handle = future.get();
    //     if(!goal_handle){
    //         RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    //     }else{
    //         RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    //     }
    // }

    void feedback_callback(
        GoalHandleCountUntil::SharedPtr,
        const std::shared_ptr<const CountUntil::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Current number : %ld", feedback->current_number);
    }

    void result_callback(const GoalHandleCountUntil::WrappedResult& result)
    {
        RCLCPP_INFO(this->get_logger(), "FINAL RESULT : %ld", result.result->reached_number);
        rclcpp::shutdown();
    }
};




int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto Node = std::make_shared<CountUntilClientNode>();
    // Node->send_goal(12, 1.0);   // it is used when I am not explicitly defining parameters using --ros-args
    
    rclcpp::spin(Node);   // it expect std::shared_ptr
    rclcpp::shutdown();
    return 0;
}
