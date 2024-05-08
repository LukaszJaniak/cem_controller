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

#ifndef CEM_CONTROLLER__CEM_CONTROLLER_HPP_
#define CEM_CONTROLLER__CEM_CONTROLLER_HPP_

#include <cstdint>

#include "cem_controller/visibility_control.hpp"

// ----------
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "trajectory_follower_base/lateral_controller_base.hpp"

#include <motion_utils/resample/resample.hpp>
#include <motion_utils/trajectory/conversion.hpp>
#include <motion_utils/trajectory/trajectory.hpp>

#include "autoware_auto_control_msgs/msg/ackermann_lateral_command.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tier4_debug_msgs/msg/float32_multi_array_stamped.hpp"

#include <boost/optional.hpp>  // To be replaced by std::optional in C++17
#include "chrono"

#include <memory>
#include <vector>

using autoware::motion::control::trajectory_follower::InputData;
using autoware::motion::control::trajectory_follower::LateralControllerBase;
using autoware::motion::control::trajectory_follower::LateralOutput;
using autoware_auto_control_msgs::msg::AckermannLateralCommand;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
// ====================================



namespace cem_controller
{

class CEM_CONTROLLER_PUBLIC CemController
{
public:
  CemController();
  int64_t foo(int64_t bar) const;
};


class CemLateralController : public LateralControllerBase
{
public:
  /// \param node Reference to the node used only for the component and parameter initialization.
  explicit CemLateralController(rclcpp::Node & node);
  int64_t fun(int64_t bar) const;

  int target_idx=15;
  double calc_distance(double x_,double y_,double x,double y);
  double dst;
  double Lf; 
  double Lfc= 1.0;
  double m_longitudinal_ctrl_period=0.03;
  // double Lfc= 2.0;
  double k;
  double dt;
  double v;
  // std::chrono::steady_clock::time_point last_run_time_;
  double yawFromPose(const geometry_msgs::msg::Pose& pose);
  double yaw;
  double cemSteerControl(const geometry_msgs::msg::Pose& state, double Lf, const autoware_auto_planning_msgs::msg::TrajectoryPoint& target_state);
  std::shared_ptr<rclcpp::Time> m_prev_control_time{nullptr};

  std::vector<double> discreteDynamics(const std::vector<double>& x, const std::vector<double>& u, double dt);

  std::vector<std::tuple<double, double, std::vector<double> >> simulateCost( const geometry_msgs::msg::Pose& state, const autoware_auto_planning_msgs::msg::TrajectoryPoint& targetState );

  double randomSteer(double mu, double sigma);
  double di;
  double mu;
  double sigma;
  double sigma_shrink;
  double converged_steer_rad_;
  double acc;
  bool calcIsSteerConverged(const AckermannLateralCommand & cmd, const autoware_auto_vehicle_msgs::msg::SteeringReport& stering,double converged_steer_rad_);
  double getAction(const std::vector<std::tuple<double, double, std::vector<double>>>& cost_tuple_list, int elite_size);
  double getDt();
private:
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_;
  bool isReady([[maybe_unused]] const InputData & input_data) override;
  LateralOutput run([[maybe_unused]] const InputData & input_data) override;


  geometry_msgs::msg::Pose current_pose_;
  autoware_auto_planning_msgs::msg::Trajectory trajectory_;
  nav_msgs::msg::Odometry current_odometry_;
  autoware_auto_vehicle_msgs::msg::SteeringReport current_steering_;
 

};


}  // namespace cem_controller

#endif  // CEM_CONTROLLER__CEM_CONTROLLER_HPP_
