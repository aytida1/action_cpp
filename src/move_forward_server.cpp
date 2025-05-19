#include <functional>
#include <memory>
#include <thread>


#include "rclcpp/rclcpp.hpp"
#include "action_template_interfaces/action/move_forward.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

class MoveForwardServer : public rclcpp::Node
{
public:
    using MoveForward = action_template_interfaces::action::MoveForward;
    using GoalHandle = rclcpp_action::ServerGoalHandle<MoveForward>;

    explicit MoveForwardServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
     : Node("move_forward_action_server", options)
    {
        using namespace std::placeholders;

        // Velocity publisher
        vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "cmd_vel",
            10
        );

        // Odometry subsciber
        odom_subsciber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "new_odom",
            10,
            std::bind(&MoveForwardServer::odom_callback, this, _1)
        );

        // creating action server
        this->action_server_ = rclcpp_action::create_server<MoveForward>(
            this->get_node_base_interface(),
            this->get_node_clock_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "MoveForward",
            std::bind(&MoveForwardServer::handle_goal, this, _1, _2),
            std::bind(&MoveForwardServer::handle_cancel, this, _1),
            std::bind(&MoveForwardServer::handle_accepted, this, _1)
        ) ;

        RCLCPP_INFO(this->get_logger(), "Action server has been started.");
        
        
    }

private:
    //defining some global parameters
    float current_x, current_y;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subsciber_;
    rclcpp_action::Server<MoveForward>::SharedPtr action_server_;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MoveForward::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Goal request received👆 : Move forward upto %f distance.", goal->target_distance);
        (void)uuid;  // to let compiler know that I know that it exist and it is not in used intensionaly.
        float t_distance = goal->target_distance;
        
        if((t_distance<=0) || (t_distance>10))
        {
            return rclcpp_action::GoalResponse::REJECT;
        }else{
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to CANCEL goal.😞");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }


    void execute(const std::shared_ptr<GoalHandle> goal_handle)
    {

        auto goal = goal_handle->get_goal();
        float target_distance = goal->target_distance;
        float initial_x = current_x;
        float initial_y = current_y;
        auto feedback = std::make_shared<MoveForward::Feedback>();
        auto & distance_travelled = feedback->distance_travelled;
        auto result = std::make_shared<MoveForward::Result>();

        //defining velocity message that will be published
        geometry_msgs::msg::Twist vel_msg;
        vel_msg.linear.x = 0;
        vel_msg.linear.y = 0;
        vel_msg.linear.z = 0;
        vel_msg.angular.x = 0;
        vel_msg.angular.y = 0;
        vel_msg.angular.z = 0;

        double prev_time = 0;
        
        while(true)
        {
            auto now = rclcpp::Clock().now();  // needs to update in every time

            float current_distance = sqrt(pow(current_x-initial_x, 2)+pow(current_y-initial_y, 2));
            if (current_distance >= target_distance){
                vel_msg.linear.x = 0.0;
                vel_publisher_->publish(vel_msg);
                break;
            } else {
                vel_msg.linear.x = 0.2;  // constant velocity
                vel_publisher_->publish(vel_msg);
            }

            if ((now.seconds()-prev_time) > 1){
                distance_travelled = current_distance;
                goal_handle->publish_feedback(feedback);
                prev_time = now.seconds();
            }
        }

        if (rclcpp::ok())
        {
            // publish result
            result->result = "😃😃😃😃Reached Distance😃😃😃😃";
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
        }
        
    }

    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
    {
        using namespace std::placeholders;
        RCLCPP_INFO(this->get_logger(), "Executing Goal😄!!!");
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&MoveForwardServer::execute, this, _1), goal_handle}.detach();
    }
};


void MoveForwardServer::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_x = msg->pose.pose.position.x;
    current_y = msg->pose.pose.position.y;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MoveForwardServer>());
    rclcpp::shutdown();
    return 0;
}