#include "geometry_msgs/msg/detail/pose__struct.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "grasping_msgs/action/detail/find_graspable_objects__struct.hpp"
#include "grasping_msgs/action/find_graspable_objects.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <string>
#include <thread>
#include <vector>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_ur3");
static const std::string PLANNING_GROUP_ARM =
    "ur_arm"; // this name must be the same in rviz
static const std::string PLANNING_GROUP_GRIPPER =
    "gripper"; // this name must be the same in rviz
using namespace std::chrono_literals;

class PickPlace : public rclcpp::Node {

public:
  using FindObj = grasping_msgs::action::FindGraspableObjects;
  using GoalHandleFindObj = rclcpp_action::ClientGoalHandle<FindObj>;

  PickPlace(std::shared_ptr<rclcpp::Node> move_group_node)
      : Node("pick_and_place"),
        move_group_arm(move_group_node, PLANNING_GROUP_ARM),
        joint_model_group_arm(
            move_group_arm.getCurrentState()->getJointModelGroup(
                PLANNING_GROUP_ARM)),
        move_group_gripper(move_group_node, PLANNING_GROUP_GRIPPER),
        joint_model_group_gripper(
            move_group_gripper.getCurrentState()->getJointModelGroup(
                PLANNING_GROUP_GRIPPER)) {
    RCLCPP_INFO(LOGGER, "Move Group Initialized");

    callback_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    this->timer_ = this->create_wall_timer(
        500ms, std::bind(&PickPlace::timer_callback, this), callback_group_);

    RCLCPP_INFO(LOGGER, "Timer Initialized");

    this->client_ptr_ = rclcpp_action::create_client<FindObj>(
        this->get_node_base_interface(), this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(), "find_objects");

    RCLCPP_INFO(LOGGER, "Perception Client Initialized");
  }

  bool is_goal_done() const { return this->goal_done_; }
  void get_info() {
    // Getting Basic Information
    RCLCPP_INFO(LOGGER, "Planning frame: %s",
                move_group_arm.getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End-effector link: %s",
                move_group_arm.getEndEffectorLink().c_str());
    RCLCPP_INFO(LOGGER, "Available Planning Groups: ");
    const auto &joint_model_group_names =
        move_group_arm.getJointModelGroupNames();
    std::copy(joint_model_group_names.begin(), joint_model_group_names.end(),
              std::ostream_iterator<std::string>(std::cout, ", "));
  }

  void current_state() {
    // Update State
    RCLCPP_INFO(LOGGER, "Get Robot Current State");

    current_state_arm = move_group_arm.getCurrentState(10);
    current_state_gripper = move_group_gripper.getCurrentState(10);

    current_state_arm->copyJointGroupPositions(this->joint_model_group_arm,
                                               this->joint_group_positions_arm);
    current_state_gripper->copyJointGroupPositions(
        this->joint_model_group_gripper, this->joint_group_positions_gripper);
  }

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
        std::bind(&PickPlace::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&PickPlace::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&PickPlace::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

  void to_home_pos() {

    RCLCPP_INFO(LOGGER, "Going Home");

    joint_group_positions_arm[0] = 0.00;
    joint_group_positions_arm[1] = -1.57;
    joint_group_positions_arm[2] = 0.00;
    joint_group_positions_arm[3] = -1.57;
    joint_group_positions_arm[4] = 0.00;
    joint_group_positions_arm[5] = 0.00;

    move_group_arm.setJointValueTarget(joint_group_positions_arm);
    RCLCPP_INFO(LOGGER, "Going Home - Joint Values Set");
    while (move_group_arm.plan(my_plan_arm) !=
           moveit::planning_interface::MoveItErrorCode::SUCCESS) {
      move_group_arm.setJointValueTarget(joint_group_positions_arm);
    }

    bool success_motion =
        (move_group_arm.plan(my_plan_arm) ==
         moveit::planning_interface::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, "Going Home - Trajectory Planned");

    move_group_arm.execute(my_plan_arm);
    RCLCPP_INFO(LOGGER, "Going Home - Trajectory Complete");
  }

  void approach() {

    RCLCPP_INFO(LOGGER, "Approaching object");

    target_pose.position.x = pose_x;
    target_pose.position.y = pose_y;
    target_pose.position.z = 0.284;
    target_pose.orientation.x = -1.0;
    target_pose.orientation.y = 0.0;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 0.0;

    move_group_arm.setPoseTarget(target_pose);

    RCLCPP_INFO(LOGGER, "Approaching - Pose Values Set");

    while (move_group_arm.plan(my_plan_arm) !=
           moveit::planning_interface::MoveItErrorCode::SUCCESS) {
      move_group_arm.setPoseTarget(target_pose);
    }
    bool success_arm = (move_group_arm.plan(my_plan_arm) ==
                        moveit::planning_interface::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, "Approaching - %s: ",
                success_arm ? "Trajectory Planned" : "Trajectory Failed");

    move_group_arm.execute(my_plan_arm);
    RCLCPP_INFO(LOGGER, "Approaching - Trajectory Complete");
  }

  void to_grasp_pos() {

    RCLCPP_INFO(LOGGER, "Grasping object");
    target_pose.position.z = target_pose.position.z - offset;
    move_group_arm.setPoseTarget(target_pose);
    RCLCPP_INFO(LOGGER, "Grasping - Pose Values Set");

    while (move_group_arm.plan(my_plan_arm) !=
           moveit::planning_interface::MoveItErrorCode::SUCCESS) {
      move_group_arm.setPoseTarget(target_pose);
    }

    bool success_arm = (move_group_arm.plan(my_plan_arm) ==
                        moveit::planning_interface::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, "Approaching - %s: ",
                success_arm ? "Trajectory Planned" : "Trajectory Failed");

    move_group_arm.execute(my_plan_arm);
    RCLCPP_INFO(LOGGER, "Approaching - Trajectory Complete");
  }

  void to_place_pos() {

    RCLCPP_INFO(LOGGER, "Going to drop the object");

    joint_group_positions_arm[0] = -3.14; // Shoulder pan
    /*
    joint_group_positions_arm[1] = joint_group_positions_arm[1]; //shoulder lift
    joint_group_positions_arm[2] = joint_group_positions_arm[2]; //elbow
    joint_group_positions_arm[3] = joint_group_positions_arm[3]; //w1
    joint_group_positions_arm[4] = joint_group_positions_arm[4]; //w2
    joint_group_positions_arm[5] = joint_group_positions_arm[5]; //w3
    */
    move_group_arm.setJointValueTarget(joint_group_positions_arm);
    RCLCPP_INFO(LOGGER, "Going Home - Joint Values Set");
    while (move_group_arm.plan(my_plan_arm) !=
           moveit::planning_interface::MoveItErrorCode::SUCCESS) {
      move_group_arm.setJointValueTarget(joint_group_positions_arm);
    }

    bool success_motion =
        (move_group_arm.plan(my_plan_arm) ==
         moveit::planning_interface::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, "Going Home - Trajectory Planned");

    move_group_arm.execute(my_plan_arm);
    RCLCPP_INFO(LOGGER, "Going Home - Trajectory Complete");
  }
  void open_gripper() {

    joint_group_positions_gripper[0] = 0.4;
    move_group_gripper.setJointValueTarget(joint_group_positions_gripper);

    bool success_gripper =
        (move_group_gripper.plan(my_plan_gripper) ==
         moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // Execute
    move_group_gripper.execute(my_plan_gripper);
  }
  void close_gripper() {

    joint_group_positions_gripper[0] = -0.7;
    move_group_gripper.setJointValueTarget(joint_group_positions_gripper);

    bool success_gripper =
        (move_group_gripper.plan(my_plan_gripper) ==
         moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // Execute
    move_group_gripper.execute(my_plan_gripper);
  }

  void timer_callback() {

    this->timer_->cancel();
    get_info();
    current_state();

    to_home_pos();
    current_state();
    send_goal();

    while (!got_perception_data) {
      std::this_thread::sleep_for(1000ms);
    }

    open_gripper();
    current_state();

    approach();
    current_state();

    to_grasp_pos();
    current_state();

    close_gripper();
    current_state();

    approach();
    current_state();

    to_place_pos();
    current_state();

    open_gripper();

    // get_info();
    rclcpp::shutdown();
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface move_group_arm;
  moveit::planning_interface::MoveGroupInterface move_group_gripper;
  geometry_msgs::msg::Pose target_pose;

  moveit::core::RobotStatePtr current_state_arm;
  moveit::core::RobotStatePtr current_state_gripper;

  const moveit::core::JointModelGroup *joint_model_group_arm;
  const moveit::core::JointModelGroup *joint_model_group_gripper;

  std::vector<double> joint_group_positions_arm;
  std::vector<double> joint_group_positions_gripper;

  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp_action::Client<FindObj>::SharedPtr client_ptr_;

  bool goal_done_ = false;
  bool got_perception_data = false;
  float offset = 0.04;
  float pose_x = 0.0;
  float pose_y = 0.0;

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
    pose_x = result.result->objects[0].object.primitive_poses[0].position.x;
    pose_y = result.result->objects[0].object.primitive_poses[0].position.y;
    RCLCPP_INFO(this->get_logger(), "X: %f", pose_x);
    RCLCPP_INFO(this->get_logger(), "Y: %f", pose_y);

    got_perception_data = true;
  }
};

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  auto move_group_node =
      rclcpp::Node::make_shared("move_group_ur3", node_options);

  rclcpp::executors::MultiThreadedExecutor motion_executor;
  auto motion_node = std::make_shared<PickPlace>(move_group_node);

  motion_executor.add_node(motion_node);
  motion_executor.spin();

  rclcpp::shutdown();
  return 0;
}