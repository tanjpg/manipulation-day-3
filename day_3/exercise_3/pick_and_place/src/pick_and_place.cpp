/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

// All source files that use ROS logging should define a file-specific
// static const rclcpp::Logger named LOGGER, located at the top of the file
// and inside the namespace with the narrowest scope (if there is one)
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  //
  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the ``JointModelGroup``. Throughout MoveIt, the terms "planning group" and "joint model group"
  // are used interchangeably.
  
  static const std::string PLANNING_GROUP = move_group_node->get_parameter("planning_group").as_string();
  static const std::string HAND_PLANNING_GROUP = move_group_node->get_parameter("end_effector_planning_group").as_string();
  static const std::string BASE_LINK = move_group_node->get_parameter("base_link").as_string();
  std::vector< std::string> gripper_fingers = move_group_node->get_parameter("end_effector_finger_links").as_string_array();
  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction;

  // The
  // :moveit_codedir:`MoveGroupInterface<moveit_ros/planning_interface/move_group_interface/include/moveit/move_group_interface/move_group_interface.h>`
  // class can be easily set up using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

  // Change planners
  // ^^^^^^^^^^^^^^^
  // 1) Pilz industrial planner
  //https://ros-planning.github.io/moveit_tutorials/doc/pilz_industrial_motion_planner/pilz_industrial_motion_planner.html
  // move_group.setPlanningPipelineId("pilz_industrial_motion_planner");
  // move_group.setPlannerId("PTP");
  // move_group.setPlannerId("LIN");
  // move_group.setPlannerId("CIRC");

  // 2) ompl
  move_group.setPlanningPipelineId("ompl");
  move_group.setPlannerId("LBKPIECEkConfigDefault");  

  // We will use the
  // :moveit_codedir:`PlanningSceneInterface<moveit_ros/planning_interface/planning_scene_interface/include/moveit/planning_scene_interface/planning_scene_interface.h>`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  RCLCPP_INFO(LOGGER, "Defualt Planner for ur_manipulator: : %s", move_group.getPlanningPipelineId().c_str());

  auto home_pose = move_group.getCurrentPose();

  // TODO : Creating a Pick Object
  // ^^^^^^^^^^^^^^^^^^^^^^^^^

  // First, let us create a moveit_msgs::msg::CollisionObject object
    RCLCPP_INFO(LOGGER, "Create a a collision object");

  // 1. Set the Frame ID of the header to move_group.getPlanningFrame()

  // 2. The id of the object is used to identify it. set it as "object"

  // 3. Define a the shape of the collision object! The object type is a shape_msgs::msg::SolidPrimitive

  // 4. Set the primitive type as BOX

  // 5. Set the box dimensions to 0.1m(x), 0.3m(y), 0.1m(z)

  // 6. Define a pose for the Collision object of type geometry_msgs::msg::Pose
  //    For this exercise, we will set the box pose a fixed distance away from the robot's end effector.
  //    To get the current geometry_msgs::msg::Pose of the end effector, use move_group.getCurrentPose().pose
  //    Set the object's pose position to be -0.4m (x), -0.1m (y) and -0.4m(z) away from the end effector.
  //    Set the object's pose orientation to be 0.0(x), 0.0(y), 0.0(z), 1.0(w)

    geometry_msgs::msg::Pose box_pose;

  // 7. Push_back the primitive shape and the primitive pose  to the collision object's 
  //    relevant vectors

  // 8. Set the collision object's operation to ADD

  // 9. Create a vector of moveit_msgs::msg::CollisionObject, and push_back the new collision_object into this vector

  // 10. Now, let's add the collision object into the planning scene through the planning_scene_interface.
  // (using a vector that could contain additional objects)
    RCLCPP_INFO(LOGGER, "Add an object into the world");

  // TODO: Pick Object (Cartesian path)
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // Let us plan a cartesian path to the pick object
  // Remember that the object pose was predefined offset values with reference to the end effector pose,
  // thus your cartesian path would just require an end pose offset equal to the object pose offset

  /* In other words, from your current position (use move_group.getCurrentPose().pose to get your robot's current end effector pose) 
  you will need to move -0.4m (x), -0.1m (y) and -0.4m(z) away to reach the object */

  // 1. Create a vector of geometry_msgs::msg::Pose . This Vector will store the cartesian waypoints

  // 2. Create a geometry_msgs::msg::Pose to move the robot in the x direction only. push_back this pose to the vector

  // 3. Create a pose to move the robot in the y direction only. push_back this pose to the vector

  // 4. Finally, Create a pose to move the robot in the z direction only. push_back this pose to the vector

  /* 5. With these waypoints, uncomment the following three lines of code and add the waypoint vector to the relevant locations.
     This allows moveit to execute the trajectory*/

  // fraction = move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
  // RCLCPP_INFO(LOGGER, "Cartesian path) (%.2f%% of path achieved)", fraction * 100.0);
  // move_group.execute(trajectory);

  // TODO: Attach Object via Moveit2 (Note that the actual gripper is not moving! but Moveit attaches the object to the link)
  // ^^^^^^^^^^^^^^^^^^^^^^^^^  
  // 1. use the attachObject method in Move group to attach the object to the link "tool0"


  // TODO: Place Object at new location (Cartesian)
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // Let us plan a cartesian path to the place location
  // We will move the object 0.2m (x) and 0.2m (y) from its current position

  // 1. Create a vector of geometry_msgs::msg::Pose . This Vector will store the cartesian waypoints

  // 2. Create a geometry_msgs::msg::Pose to move the robot in the x direction only. push_back this pose to the vector

  // 3. Create a pose to move the robot in the y direction only. push_back this pose to the vector

  /* 4. With these waypoints, uncomment the following three lines of code and add the waypoint vector to the relevant locations.
     This allows moveit to execute the trajectory*/

  // fraction = move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
  // RCLCPP_INFO(LOGGER, "Cartesian path) (%.2f%% of path achieved)", fraction * 100.0);
  // move_group.execute(trajectory);

  // TODO: Release Object
  // ^^^^^^^^^^^^^^
  // 1. use the detachObject method in Move group to release the object.

  // Return To Home
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  std::vector<geometry_msgs::msg::Pose> return_home_waypoints;
  geometry_msgs::msg::Pose return_home_pose;
  return_home_pose = move_group.getCurrentPose().pose;
  return_home_pose.position.z = home_pose.pose.position.z;
  return_home_waypoints.push_back(return_home_pose);
  return_home_waypoints.push_back(home_pose.pose);

  fraction = move_group.computeCartesianPath(return_home_waypoints, 0.01, 0.0, trajectory);
  RCLCPP_INFO(LOGGER, "Cartesian path) (%.2f%% of path achieved)", fraction * 100.0);
  move_group.execute(trajectory);

  rclcpp::shutdown();
  return 0;
}