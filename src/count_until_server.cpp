#include <functional>
#include <memory>
#include <thread>


#include "rclcpp/rclcpp.hpp"
#include "action_template_interfaces/action/count_until.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class CountUntilServer : public rclcpp::Node
{
public:
    using CountUntil = action_template_interfaces::action::CountUntil;
    using GoalHandleCountUntil = rclcpp_action::ServerGoalHandle<CountUntil>;

    explicit CountUntilServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
     : Node("count_until_action_server", options)
    {
        using namespace std::placeholders;

        // creating action server
        this->action_server_ = rclcpp_action::create_server<CountUntil>(
            this->get_node_base_interface(),
            this->get_node_clock_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "CountUntil",
            std::bind(&CountUntilServer::handle_goal, this, _1, _2),
            std::bind(&CountUntilServer::handle_cancel, this, _1),
            std::bind(&CountUntilServer::handle_accepted, this, _1)
        ) ;

        RCLCPP_INFO(this->get_logger(), "Action server has been started.");
        
        
    }

private:
    rclcpp_action::Server<CountUntil>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const CountUntil::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Goal request received👆 : count until %ld with period %f", goal->target_number, goal->period);
        (void)uuid;  // to let compiler know that I know that it exist and it is not in used intensionaly.
        int t_number = goal->target_number;
        float period = goal->period;
        if((t_number<=0) || (t_number>20) || (period>5))
        {
            return rclcpp_action::GoalResponse::REJECT;
        }else{
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleCountUntil> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to CANCEL goal.😞");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }


    void execute(const std::shared_ptr<GoalHandleCountUntil> goal_handle)
    {
        
        auto goal = goal_handle->get_goal();
        int target_no = goal->target_number;
        float period_time = goal->period;
        auto feedback = std::make_shared<CountUntil::Feedback>();
        auto & current_no = feedback->current_number;
        auto result = std::make_shared<CountUntil::Result>();

        // loop rate
        rclcpp::Rate loop_rate(1/period_time);

        int count = 0;
        for(int i=1; i<=target_no; i++){
            if(goal_handle->is_canceling())
            {
                result->reached_number = i;
                goal_handle->canceled(result);
            }
            // publishing feedback
            current_no = i;
            goal_handle->publish_feedback(feedback);
            
            count += 1;
            loop_rate.sleep();
        }

        // publishing result
        if (rclcpp::ok())
        {
            result->reached_number = count;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
        }
    }

    void handle_accepted(const std::shared_ptr<GoalHandleCountUntil> goal_handle)
    {
        using namespace std::placeholders;
        RCLCPP_INFO(this->get_logger(), "Executing Goal😄!!!");
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&CountUntilServer::execute, this, _1), goal_handle}.detach();
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CountUntilServer>());
    rclcpp::shutdown();
    return 0;
}