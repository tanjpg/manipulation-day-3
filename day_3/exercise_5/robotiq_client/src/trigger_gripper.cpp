// Copyright 2021 ROS Industrial Consortium Asia Pacific
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
#include <unistd.h>

static const rclcpp::Logger & LOGGER = rclcpp::get_logger("Service Trigger");

using namespace std::chrono_literals;
void activate_gripper(
  rclcpp::Node::SharedPtr demo_node,
  int gripper_width){
  
  //TODO: Write a Service Client call to call the "gripper_service" server
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.allow_undeclared_parameters(true);
  node_options.automatically_declare_parameters_from_overrides(true);

  rclcpp::Node::SharedPtr gripper_node =
    rclcpp::Node::make_shared("gripper_client", "", node_options);

  int curr_gripper_width = 15;
  
  // loop from grip width of 15cm(closed) to 85 cm (open)
  while (rclcpp::ok) {
    activate_gripper(
      gripper_node,
      curr_gripper_width);
    
    if(curr_gripper_width == 85){
      curr_gripper_width = 15;
    }else{
      curr_gripper_width+= 10;
    }
    sleep(1);
  }
  rclcpp::shutdown();
  return 0;
}
