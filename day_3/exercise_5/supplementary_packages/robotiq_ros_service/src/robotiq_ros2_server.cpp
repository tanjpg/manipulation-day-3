// Copyright 2023 ROS Industrial Consortium Asia Pacific
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "robotiq_msgs/srv/c_model_response.hpp"

namespace gripper_service
{
class RobotiqGripperServer : public rclcpp::Node
{
public:
  RobotiqGripperServer()
  : Node("gripper_server",
      rclcpp::NodeOptions()
      .allow_undeclared_parameters(true)
      .automatically_declare_parameters_from_overrides(true))
  {
    auto fake_gripper_service = [this](
      const std::shared_ptr<robotiq_msgs::srv::CModelResponse::Request> request,
      std::shared_ptr<robotiq_msgs::srv::CModelResponse::Response> response) ->
      void {
        // Request information for moving gripper
        int configuration_type = request->configuration;
        bool isEpick = configuration_type == 2 ? true : false;
        int grip_type = request->robotiq2f_type;
        float target_position = static_cast<float>(request->rpr);

        // Print out request information
        RCLCPP_INFO(
          this->get_logger(),
          "%sConfig:%s\nrobotiq2f_type:%s\nObj\nWidth: %f | Spd: %i | Force: %i\nePick mode:%s%s",
          "\n============== Service requested ==============\n",
          configuration_type == 1 ? " Robotiq 2f" : " ePick",
          isEpick ? "-" : (grip_type == 85 ? " Robotiq 85" : " Robotiq 140"),
          target_position,
          request->rsp,
          request->rfr,
          request->rmod == 0 ? " Auto" : " Advanced",
          "\n===============================================");
        
        // Print request information 
        RCLCPP_WARN(
          this->get_logger(),
          "Target Gripper state: %s",
          target_position == robotiq_gripper_max_width ? "Open" : "Close"
        );

        // For Finger Grippers, publish the joint changes based on the gripper width
        if (configuration_type == 1) {
          // Set normalised position
          auto remap_pos = 1.0 - (target_position / robotiq_gripper_max_width);
          set_position(remap_pos, grip_type);
          response->obj_detected = true;    // Reply with true
        } else {
          response->obj_detected = false;
        }
      };

    // Create service for gripper
    service_ = this->create_service<robotiq_msgs::srv::CModelResponse>(
      this->get_parameter("client_request_service").as_string(), fake_gripper_service);

    RCLCPP_INFO(
      this->get_logger(),
      "Started gripper fake service server..."
    );

    // Create publisher to publish joint states to the joint topic
    joint_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      this->get_parameter("trajectory_topic").as_string(), 10);
  }

private:
  const int robotiq_gripper_max_width {85};
  rclcpp::Service<robotiq_msgs::srv::CModelResponse>::SharedPtr service_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_publisher_;
  bool sim_only;

  void set_position(float target_position, int grip_type)
  {
    // Sent trajectory command to the controller
    trajectory_msgs::msg::JointTrajectory command_out_traj_;
    int dof {1};
    command_out_traj_.header.stamp = rclcpp::Time(0);
    if (grip_type == 85) {
      command_out_traj_.joint_names.push_back(this->get_parameter("robotiq_85_joint").as_string());
    } else {
      command_out_traj_.joint_names.push_back(this->get_parameter("robotiq_140_joint").as_string());
    }
    command_out_traj_.points.resize(1);
    command_out_traj_.points.front().positions.resize(dof);
    command_out_traj_.points.front().positions[0] = target_position;
    command_out_traj_.points.front().velocities.resize(dof);
    // command_out_traj_.points.front().effort.resize(dof);
    // To be reached 1 second after starting along the traj
    command_out_traj_.points.front().time_from_start = rclcpp::Duration(
      std::chrono::milliseconds(500));
    joint_publisher_->publish(command_out_traj_);
  }
};
}  // namespace gripper_service

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<gripper_service::RobotiqGripperServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}

