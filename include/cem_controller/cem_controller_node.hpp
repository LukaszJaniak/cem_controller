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

#ifndef CEM_CONTROLLER__CEM_CONTROLLER_NODE_HPP_
#define CEM_CONTROLLER__CEM_CONTROLLER_NODE_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "cem_controller/cem_controller.hpp"

namespace cem_controller
{
using CemControllerPtr = std::unique_ptr<cem_controller::CemController>;

class CEM_CONTROLLER_PUBLIC CemControllerNode : public rclcpp::Node
{
public:
  explicit CemControllerNode(const rclcpp::NodeOptions & options);

private:
  CemControllerPtr cem_controller_{nullptr};
  int64_t param_name_{123};
};
}  // namespace cem_controller

#endif  // CEM_CONTROLLER__CEM_CONTROLLER_NODE_HPP_
