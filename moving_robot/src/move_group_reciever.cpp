#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/msg/point.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

class MoveGroupReceiver : public rclcpp::Node {
public:
    MoveGroupReceiver() : Node("move_group_reciever") {
        // Create the subscriber
        position_subscriber_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/target_position", 10,
            [this](geometry_msgs::msg::Point::SharedPtr msg) {
                target_pose1.position.x = msg->x;  
                target_pose1.position.y = msg->y;  
                target_pose1.position.z = msg->z;
                target_pose1.orientation.w = 1.0;

                // Unsubscribe after processing the first message
                position_subscriber_.reset(); // This stops the subscription
            }
        );
    }

    geometry_msgs::msg::Pose getTargetPose() {
        return target_pose1;
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr position_subscriber_;
    geometry_msgs::msg::Pose target_pose1;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_reciever = std::make_shared<MoveGroupReceiver>();

    // Single-threaded executor for the robot state monitor
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_reciever);
    std::thread([&executor]() { executor.spin(); }).detach();

    // Set the planning group for UR manipulator
    static const std::string PLANNING_GROUP = "ur_manipulator";

    // Initialize MoveGroupInterface with UR manipulator planning group
    moveit::planning_interface::MoveGroupInterface move_group(move_group_reciever, PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // Visualization setup
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools(move_group_reciever, "base_link", "rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update",
                                                        move_group.getRobotModel());

    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 0.5;
    visual_tools.publishText(text_pose, "UR Manipulator Demo", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    // Display basic information about the robot
    RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());
    RCLCPP_INFO(LOGGER, "Available Planning Groups:");
    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
              std::ostream_iterator<std::string>(std::cout, ", "));

    // Wait for the subscriber to receive the first message
    rclcpp::sleep_for(std::chrono::seconds(1)); // Allow time for the subscriber to process

    // Get the target pose after the first message is received
    geometry_msgs::msg::Pose target_pose1 = move_group_reciever->getTargetPose();
    move_group.setPoseTarget(target_pose1);

    move_group.setMaxVelocityScalingFactor(0.2);
    move_group.setMaxAccelerationScalingFactor(0.2);

    // Plan and visualize
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, "Visualizing plan (pose goal) %s", success ? "" : "FAILED");

    visual_tools.publishAxisLabeled(target_pose1, "pose1");
    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    move_group.move();

    const char* command = "ros2 action send_goal -f /robotiq_2f_urcap_adapter/gripper_command robotiq_2f_urcap_adapter/GripperCommand '{ command: { position: 0.01, max_effort: 140, max_speed: 0.15 }}'";
    int result = system(command);
    if (result == 0) {
        std::cout << "Command executed successfully." << std::endl;
    } else {
        std::cout << "Command execution failed." << std::endl;
    }

  // Planning to a joint-space goal (as before)
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions[5] = -2;  // Modify this for UR robot
    bool within_bounds = move_group.setJointValueTarget(joint_group_positions);
    if (!within_bounds)
    {
    RCLCPP_WARN(LOGGER, "Target joint position(s) were outside of limits, but will be clamped.");
    }

    move_group.setMaxVelocityScalingFactor(0.2);
    move_group.setMaxAccelerationScalingFactor(0.2);

    success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, "Visualizing plan (joint-space goal) %s", success ? "" : "FAILED");

    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    move_group.move();

  // Planning to a joint-space goal (as before)
    current_state = move_group.getCurrentState(10);
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions[0] = -3.14;
    joint_group_positions[1] = -1.72;
    joint_group_positions[2] = -2.20;
    joint_group_positions[3] = -0.802;
    joint_group_positions[4] = 1.57;
    joint_group_positions[5] = -0;  // Modify this for UR robot
    within_bounds = move_group.setJointValueTarget(joint_group_positions);
    if (!within_bounds)
    {
    RCLCPP_WARN(LOGGER, "Target joint position(s) were outside of limits, but will be clamped.");
    }

    move_group.setMaxVelocityScalingFactor(0.2);
    move_group.setMaxAccelerationScalingFactor(0.2);

    success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, "Visualizing plan (joint-space goal) %s", success ? "" : "FAILED");

    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    move_group.move();

    command = "ros2 action send_goal -f /robotiq_2f_urcap_adapter/gripper_command robotiq_2f_urcap_adapter/GripperCommand '{ command: { position: 0.082, max_effort: 140, max_speed: 0.15 }}'";
    result = system(command);
    if (result == 0) {
        std::cout << "Command executed successfully." << std::endl;
    } else {
        std::cout << "Command execution failed." << std::endl;
    }
    // New Cartesian Path to Another Location (Second Cartesian Path)
    // geometry_msgs::msg::Pose target_pose2;
    // target_pose1.orientation.w = 1.0;
    // target_pose1.position.x = 0.446;  // Set according to UR robot's workspace
    // target_pose1.position.y = -0.093;
    // target_pose1.position.z = 0.504;

    // move_group.setPoseTarget(target_pose2);
    // move_group.setMaxVelocityScalingFactor(0.2);
    // move_group.setMaxAccelerationScalingFactor(0.2);

    // // Plan and visualize for the second target pose
    // success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    // RCLCPP_INFO(LOGGER, "Visualizing plan (pose goal to another location) %s", success ? "" : "FAILED");

    // visual_tools.publishAxisLabeled(target_pose2, "pose2");
    // visual_tools.publishText(text_pose, "Moving to Another Pose", rvt::WHITE, rvt::XLARGE);
    // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    // visual_tools.trigger();
    // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    // move_group.move();

  //Cartesian Path example
  // std::vector<geometry_msgs::msg::Pose> waypoints;
  // geometry_msgs::msg::Pose start_pose2 = target_pose1;
  // waypoints.push_back(start_pose2);

  // geometry_msgs::msg::Pose target_pose2 = start_pose2;
  // target_pose2.position.z -= 0.2;  // Move down
  // waypoints.push_back(target_pose2);

  // target_pose2.position.y -= 0.2;  // Move right
  // waypoints.push_back(target_pose2);

  // target_pose2.position.z += 0.2;
  // target_pose2.position.y += 0.2;
  // target_pose2.position.x -= 0.2;  // Move up and left
  // waypoints.push_back(target_pose2);

  // move_group.setMaxVelocityScalingFactor(0.05);
  // move_group.setMaxAccelerationScalingFactor(0.05);

  // moveit_msgs::msg::RobotTrajectory trajectory;
  // const double eef_step = 0.01;
  // const double jump_threshold = 1;
  // double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  // RCLCPP_INFO(LOGGER, "Visualizing Cartesian path (%.2f%% achieved)", fraction * 100.0);

  // visual_tools.deleteAllMarkers();
  // visual_tools.publishText(text_pose, "Cartesian Path", rvt::WHITE, rvt::XLARGE);
  // visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  // for (size_t i = 0; i < waypoints.size(); ++i)
  //   visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  
  // visual_tools.trigger();
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  // move_group.execute(trajectory);

  //Shut down the node after demo
    rclcpp::shutdown();
    return 0;
}

