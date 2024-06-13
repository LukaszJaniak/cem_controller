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

#include "cem_controller/cem_controller.hpp"
#include <chrono>

#include <iostream>
#include <random>

namespace cem_controller
{

CemController::CemController()
{
}

int64_t CemController::foo(int64_t bar) const
{
  std::cout << "Hello World, " << bar << std::endl;
  return bar;
}


CemLateralController::CemLateralController(rclcpp::Node & node)
: clock_(node.get_clock()),
logger_(node.get_logger().get_child("lateral_controller"))
{
  converged_steer_rad_ = node.declare_parameter<double>("converged_steer_rad");
}

bool CemLateralController::isReady([[maybe_unused]] const InputData & input_data)
{
  return true;
}

LateralOutput CemLateralController::run( [[maybe_unused]] const InputData & input_data)
{
  current_pose_ = input_data.current_odometry.pose.pose;
  trajectory_ = input_data.current_trajectory;
  current_odometry_ = input_data.current_odometry;
  current_steering_ = input_data.current_steering;

  const auto input_tp_array = motion_utils::convertToTrajectoryPointArray(trajectory_);
  int array_size = static_cast<int>(input_tp_array.size());

  dst = calc_distance(input_tp_array[target_idx].pose.position.x, input_tp_array[target_idx].pose.position.y, current_pose_.position.x, current_pose_.position.y ) ;

  Lfc = std::max( 1.2, Lfc * (current_odometry_.twist.twist.linear.x / 10.0));
  k = std::max(0.065, k * (current_odometry_.twist.twist.linear.x / 10.0));
  Lf = Lfc + k * current_odometry_.twist.twist.linear.x;

  if(v!=current_odometry_.twist.twist.linear.x){
    acc = (current_odometry_.twist.twist.linear.x - v) /dt;
  }

  v=current_odometry_.twist.twist.linear.x;

  while (dst < Lf ) {
    target_idx++;
    if (target_idx >= array_size){
        target_idx =0;
    }
    dst = calc_distance(input_tp_array[target_idx].pose.position.x, input_tp_array[target_idx].pose.position.y, current_pose_.position.x, current_pose_.position.y ) ;
  } 


  yaw=yawFromPose(current_pose_);
  di=cemSteerControl(current_pose_, Lf, input_tp_array[target_idx]);

  AckermannLateralCommand  cmd_msg;
  LateralOutput output;

  cmd_msg.stamp = clock_->now();
  cmd_msg.steering_tire_angle=di;
  output.control_cmd = cmd_msg;

  return output;
}


double CemLateralController::getDt()
{
  double dt;
  if (!m_prev_control_time) {
    dt = m_longitudinal_ctrl_period;
    m_prev_control_time = std::make_shared<rclcpp::Time>(clock_->now());
  } else {
    dt = (clock_->now() - *m_prev_control_time).seconds();
    *m_prev_control_time = clock_->now();
  }
  const double max_dt = m_longitudinal_ctrl_period * 2.0;
  const double min_dt = m_longitudinal_ctrl_period * 0.5;
  return std::max(std::min(dt, max_dt), min_dt);
}




double CemLateralController::calc_distance(double x_,double y_,double x,double y) 
{
    double dx = x_ - x; 
    double dy = y_ - y; 
    return std::sqrt(dx * dx + dy * dy); 
}

double CemLateralController::yawFromPose(const geometry_msgs::msg::Pose& pose) {

    const auto& quaternion = pose.orientation;

    return std::atan2(2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
                      quaternion.w * quaternion.w + quaternion.x * quaternion.x -
                      quaternion.y * quaternion.y - quaternion.z * quaternion.z);
}


double CemLateralController::cemSteerControl(const geometry_msgs::msg::Pose& state, double Lf, const autoware_auto_planning_msgs::msg::TrajectoryPoint& target_state) {
    const int iter_cem = 16; 
    const int elite_size = 7; 
    sigma = 1.0; 

    double delta = 2.0;
    if (Lf == 0) {
        delta = 0.0; 
    }else{
        std::vector<std::tuple<double, double, std::vector<double> >>  cost_tuple_list; 
        for (int iteration = 0; iteration < iter_cem; iteration++) {

            cost_tuple_list = simulateCost(state, target_state);
            delta = getAction(cost_tuple_list, elite_size);
        }
    }
    mu = 0.0;
    return delta; 
}

double CemLateralController::getAction(const std::vector<std::tuple<double, double, std::vector<double>>>& cost_tuple_list, int elite_size) {
    std::vector<double> elites;

    for (const auto& tuple_item : cost_tuple_list) {
        elites.push_back(std::get<1>(tuple_item));
    }


    elites.resize(elite_size);

    double average = std::accumulate(elites.begin(), elites.end(), 0.0) / elites.size();

    mu = average;
    mu = std::max(-0.62 + (sigma / 2), std::min(0.62 - (sigma / 2), mu));
    sigma *= sigma_shrink;

    return average;
}

std::vector<double> CemLateralController::discreteDynamics(const std::vector<double>& x, const std::vector<double>& u, double dt) {

    double yaw = x[2];
    double v = x[3];
    double acceleration = u[0];
    double steering = u[1];

    steering = std::min(std::max(steering, -0.62), 0.62); 

    std::vector<double> x_next{
        v * std::cos(yaw),
        v * std::sin(yaw),
        v * std::tan(steering) / 0.28,
        acceleration
    };

    std::vector<double> x_new;
    for (size_t i = 0; i < x.size(); ++i) {
        x_new.push_back(x[i] + dt * x_next[i]);
    }

    return x_new;
}


std::vector<std::tuple<double, double, std::vector<double>>> CemLateralController::simulateCost( const geometry_msgs::msg::Pose& state, const autoware_auto_planning_msgs::msg::TrajectoryPoint& targetState ) {

    const int iter_cem = 13; 
    const double cost_err = 0.9;  
    const int traj_dim = 17;

    std::vector<double> sim_state;
    std::vector<std::tuple<double, double, std::vector<double> >> cost_tuple_list;
    std::vector<double> x0 = {state.position.x, state.position.y, yaw, v}; // 

    for (int iteration = 0; iteration < iter_cem; ++iteration) {

        double steer_angle = randomSteer(mu, sigma);
       
        std::vector<std::vector<double>> u_trj(traj_dim, std::vector<double>(2, 0.0));
        for (int i = 0; i < traj_dim; ++i) {
            u_trj[i][0] = acc; // 
            u_trj[i][1] = steer_angle;
        }


        std::vector<std::vector<double>> x_trj(traj_dim + 1, std::vector<double>(x0.size(), 0.0));
        x_trj[0] = x0;

        double cost = 999999999.9; 


        for (int n = 0; n < traj_dim; ++n) {

            std::vector<double> x_next = discreteDynamics(x_trj[n], u_trj[n], dt);
            sim_state=x_next;
 
            double cost_new = sqrt(pow(x_next[0] - targetState.pose.position.x, 2) + pow(x_next[1] - targetState.pose.position.y, 2));


            if (cost_new < cost) {
                cost = cost_new;
            }

            if (cost <= cost_err) {
                break;
            }

            x_trj[n + 1] = x_next;
        }

        cost_tuple_list.push_back(make_tuple(cost, steer_angle, sim_state));
    }
    std::sort(cost_tuple_list.begin(), cost_tuple_list.end());
    return cost_tuple_list;
}

double CemLateralController::randomSteer(double mu, double sigma) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> distribution(mu, sigma);

    double rand_steer = distribution(gen);

    return rand_steer;
}

bool CemLateralController::calcIsSteerConverged(const AckermannLateralCommand & cmd, const autoware_auto_vehicle_msgs::msg::SteeringReport& stering,double converged_steer_rad_)
{
  return std::abs(cmd.steering_tire_angle - stering.steering_tire_angle) <
         static_cast<float>(converged_steer_rad_);
}

}  // namespace cem_controller


