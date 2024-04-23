// Copyright 2024 MAINTAINER
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "cem_controller/cem_controller_node.hpp"

namespace cem_controller
{

CemControllerNode::CemControllerNode(const rclcpp::NodeOptions & options)
:  Node("cem_controller", options)
{
  cem_controller_ = std::make_unique<cem_controller::CemController>();
  param_name_ = this->declare_parameter("param_name", 456);
  cem_controller_->foo(param_name_);
}

}  // namespace cem_controller

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(cem_controller::CemControllerNode)
