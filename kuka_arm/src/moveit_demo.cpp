#include <string>
#include <sstream>
#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

using namespace std;

static void moveit_demo(ros::NodeHandle nh) {
  const string PLANNING_GROUP = "arm_group";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const robot_state::JointModelGroup *joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("world");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 4.0;
  visual_tools.publishText(text_pose, "Moveit Demo", rvt::WHITE, rvt::XXXXLARGE);
  visual_tools.trigger();

  while (ros::ok()) {
    visual_tools.prompt("next step");

    float target_x, target_y, target_z;
    float bin_x, bin_y, bin_z;
    ros::param::get("/target_spawn_location/x", target_x);
    ros::param::get("/target_spawn_location/y", target_y);
    ros::param::get("/target_spawn_location/z", target_z);

    ros::param::get("/target_drop_location/x", bin_x);
    ros::param::get("/target_drop_location/y", bin_y);
    ros::param::get("/target_drop_location/z", bin_z);

    geometry_msgs::Pose target_pose, bin_pose;
    target_pose.orientation.w = 1.0;
    target_pose.position.x = target_x - 0.4;
    target_pose.position.y = target_y;
    target_pose.position.z = target_z - 0.1;

    bin_pose.orientation.w = 1.0;
    bin_pose.position.x = bin_x - 0.1;
    bin_pose.position.y = bin_y;
    bin_pose.position.z = bin_z + 1.6;

    // Plan to pick target
    move_group.setStartStateToCurrentState();
    move_group.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan to target: %s", success ? "SUCCEEDED" : "FAILED");

    // Visualize the plan
    visual_tools.publishAxisLabeled(target_pose, "target_pose");
    visual_tools.publishText(text_pose, "Displaying plan to target location", rvt::WHITE, rvt::XXXXLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Next step");

    visual_tools.publishText(text_pose, "Moving to target location", rvt::WHITE, rvt::XXXXLARGE);
    success = (move_group.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Moving to pick location: %s", success ? "SUCCEEDED" : "FAILED");
    visual_tools.trigger();
    visual_tools.prompt("Next step");

    // Plan to bin location for drop-off
    move_group.setStartStateToCurrentState();
    move_group.setPoseTarget(bin_pose);
    visual_tools.publishText(text_pose, "Displaying plan to drop-off location", rvt::WHITE, rvt::XXXXLARGE);
    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan to drop location: %s", success ? "SUCCEEDED" : "FAILED");
    visual_tools.trigger();
    visual_tools.prompt("Next step");

    visual_tools.publishText(text_pose, "Moving to drop-off location", rvt::WHITE, rvt::XXXXLARGE);
    success = (move_group.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Moving to drop-off location: %s", success ? "SUCCEEDED" : "FAILED");
    visual_tools.trigger();
    visual_tools.prompt("Next step");
  }
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "moveit_demo");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit_demo(node_handle);

  ros::shutdown();
  return 0;
}