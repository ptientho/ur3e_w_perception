

#include "grasping_msgs/action/detail/find_graspable_objects__struct.hpp"
#include "grasping_msgs/action/find_graspable_objects.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_action/client.hpp"
#include "rclcpp_action/client_goal_handle.hpp"
#include "rclcpp_action/create_client.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <functional>
#include <future>
#include <memory>

using namespace std::chrono_literals;

class GetPoseClient : public rclcpp::Node {

public:
  using FindObj = grasping_msgs::action::FindGraspableObjects;
  using GoalHandleFindObj = rclcpp_action::ClientGoalHandle<FindObj>;

  explicit GetPoseClient(const rclcpp::NodeOptions &opt = rclcpp::NodeOptions())
      : Node("minimal_action_client", opt), goal_done_(false) {
    this->client_ptr_ = rclcpp_action::create_client<FindObj>(
        this->get_node_base_interface(), this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(), "find_objects");

    this->timer_ = this->create_wall_timer(
        500ms, std::bind(&GetPoseClient::send_goal, this));
  }

  bool is_goal_done() const { return this->goal_done_; }

  void send_goal() {

    using namespace std::placeholders;
    this->timer_->cancel();
    this->goal_done_ = false;
    //~~~~~~~~~~~~Wait for server~~~~~~~~~~~~~
    if (!this->client_ptr_->wait_for_action_server(10s)) {
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");
      this->goal_done_ = true;
      return;
    }
    //~~~~~~~~~~~~Goal message~~~~~~~~~~~~~~~
    auto goal_msg = FindObj::Goal();
    goal_msg.plan_grasps = false;
    //~~~~~~~~~~~~Send Goal~~~~~~~~~~~~~~~~~~
    RCLCPP_INFO(this->get_logger(), "Sending Goal");
    auto send_goal_options = rclcpp_action::Client<FindObj>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&GetPoseClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&GetPoseClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&GetPoseClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  bool goal_done_;
  rclcpp_action::Client<FindObj>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  //~~~~~~~~~~~~~~~~Goal Callback~~~~~~~~~~~~~~~~~~~~~~
  void goal_response_callback(
      std::shared_future<GoalHandleFindObj::SharedPtr> future) {

    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Goal accepted by server, waiting for result");
    }
  }
  //~~~~~~~~~~~~~~~Feedback Callback~~~~~~~~~~~~~~~~~~~~~
  void
  feedback_callback(GoalHandleFindObj::SharedPtr,
                    const std::shared_ptr<const FindObj::Feedback> feedback) {

    RCLCPP_INFO(this->get_logger(), "Ignoring feedback...");
  }
  //~~~~~~~~~~~~~~Result Callback~~~~~~~~~~~~~~~~~~~~~~~
  void result_callback(const GoalHandleFindObj::WrappedResult &result) {

    this->goal_done_ = true;
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Result received");
    RCLCPP_INFO(this->get_logger(), "X: %f",
                result.result->objects[0].object.primitive_poses[0].position.x);
    RCLCPP_INFO(this->get_logger(), "Y: %f",
                result.result->objects[0].object.primitive_poses[0].position.y);

    //RCLCPP_INFO(this->get_logger(), "X,Y published!");           
  }
};

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  auto action_client = std::make_shared<GetPoseClient>();

  while (!action_client->is_goal_done()) {
    rclcpp::spin_some(action_client);
  }

  rclcpp::shutdown();
  return 0;
}