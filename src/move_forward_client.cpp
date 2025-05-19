#include <functional>
#include <memory>
#include <thread>
#include <chrono>
#include <future>


#include "rclcpp/rclcpp.hpp"
#include "action_template_interfaces/action/move_forward.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"


class MoveForwardClient : public rclcpp::Node
{
public:
    using MoveForward = action_template_interfaces::action::MoveForward;
    using GoalHandle = rclcpp_action::ClientGoalHandle<MoveForward>;

    explicit MoveForwardClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
     : Node("move_forward_action_client", node_options)
    {
        this->declare_parameter("target_distance", 5.0);

        // creating action client
        this->action_client_ = rclcpp_action::create_client<MoveForward>(
            this->get_node_base_interface(),
            this->get_node_graph_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(), 
            "MoveForward");
        RCLCPP_INFO(this->get_logger(), "Action client has been started.");

        // Start the goal sending process after a short delay to allow service discovery
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MoveForwardClient::send_goal_from_parameters, this));
    }

    void send_goal_from_parameters()
    {
        // Cancel the timer to prevent multiple calls
        timer_->cancel();
        
        // Get parameters
        float target_distance = float(this->get_parameter("target_distance").as_double());
        
        
        RCLCPP_INFO(this->get_logger(), "Sending goal with target_distance: %.2f", target_distance);
                    
        send_goal(target_distance);
    }
 
    void send_goal(float t_distance)
    {
        using namespace std::placeholders;  // Add this line for _1, _2 placeholders

        // wait for action server and it not available then exiting the node
        if(!this->action_client_->wait_for_action_server(std::chrono::seconds(10)))
        {
            RCLCPP_INFO(this->get_logger(), "After wait action server is not available. Sorry!!!");
            rclcpp::shutdown();
            return;
        }

        // defining goal
        auto goal_msg = MoveForward::Goal();
        goal_msg.target_distance = t_distance;

        // define callbacks for goal response, feedback and result
        auto send_goal_options = rclcpp_action::Client<MoveForward>::SendGoalOptions();
        // send_goal_options.goal_response_callback = std::bind(&CountUntilClientNode::goal_response_callback, this, _1);
        send_goal_options.goal_response_callback = [this](const GoalHandle::SharedPtr & goal_handle)
        {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
        };
        send_goal_options.feedback_callback = std::bind(&MoveForwardClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback = std::bind(&MoveForwardClient::result_callback, this, _1);
        this->action_client_->async_send_goal(goal_msg, send_goal_options);

    }

private:
    rclcpp_action::Client<MoveForward>::SharedPtr action_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    void feedback_callback(
        GoalHandle::SharedPtr,
        const std::shared_ptr<const MoveForward::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Distance Travelled : %f", feedback->distance_travelled);
    }

    void result_callback(const GoalHandle::WrappedResult& result)
    {
        RCLCPP_INFO(this->get_logger(), "%s", (result.result->result).c_str());
        rclcpp::shutdown();
    }
};




int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto Node = std::make_shared<MoveForwardClient>();
    // Node->send_goal(12);   // it is used when I am not explicitly defining parameters using --ros-args
    
    rclcpp::spin(Node);   // it expect std::shared_ptr
    rclcpp::shutdown();
    return 0;
}
