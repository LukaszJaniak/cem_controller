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


// =====================
// ros2 launch f1tenth_launch e2e_simulator.launch.py
//colcon build --packages-select
//cp -r autoware_map /root
CemLateralController::CemLateralController(rclcpp::Node & node)
: clock_(node.get_clock()),
logger_(node.get_logger().get_child("lateral_controller"))
{
  converged_steer_rad_ = node.declare_parameter<double>("converged_steer_rad");
  mu = 0.0; 
  sigma = 0.9;
  sigma_shrink = 0.85;
  yaw = 0.0;
  v = 0.0;
  acc=0.0;
  dt= 0.005;
  k= 0.1;//wcześniej 0.008
  //   dt= 0.0055;
  // k= 0.001;
}
// jedno z lepszych ale slam zile działa
  // dt= 0.0055;
  // k= 0.01;
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


  // RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000,"CEMMMMMMMMMMMMM3333-------------CEMMMMMMMMMM3333!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" );
  const auto input_tp_array = motion_utils::convertToTrajectoryPointArray(trajectory_);
  int array_size = static_cast<int>(input_tp_array.size());
  // for (const auto& point : input_tp_array) {
  //   RCLCPP_INFO(logger_, "Point: x=%f, y=%f, z=%f", point.pose.position.x, point.pose.position.y, point.pose.position.z);
  // } 


// ============
  dst = calc_distance(input_tp_array[target_idx].pose.position.x, input_tp_array[target_idx].pose.position.y, current_pose_.position.x, current_pose_.position.y ) ;
  Lf = Lfc + k * current_odometry_.twist.twist.linear.x;
  acc = (current_odometry_.twist.twist.linear.x - v) /0.0055;
  // RCLCPP_INFO(logger_, "acceleration YYYYYYYYY: %f", acc);

  v=current_odometry_.twist.twist.linear.x;
  // RCLCPP_INFO(logger_, "velocity YYYYYYYYY: %f", v);

  while (dst < Lf ) {
    target_idx++;
    if (target_idx >= array_size){
        target_idx =0;
    }
    dst = calc_distance(input_tp_array[target_idx].pose.position.x, input_tp_array[target_idx].pose.position.y, current_pose_.position.x, current_pose_.position.y ) ;
    // RCLCPP_INFO(logger_, "Wartość zmiennej while!!!!!!: %f", dst);
  } 
//   RCLCPP_INFO(logger_, "target XXXXX!!!!!!: %f", input_tp_array[target_idx].pose.position.x);
//   RCLCPP_INFO(logger_, "curent position XXXXX!!!!!!: %f", current_pose_.position.x);

//   RCLCPP_INFO(logger_, "target YYYYYYYYY: %f", input_tp_array[target_idx].pose.position.y);
//   RCLCPP_INFO(logger_, "curent position YYYYYYYYYYY: %f", current_pose_.position.y);


  yaw=yawFromPose(current_pose_);

  di=cemSteerControl(current_pose_, Lf, input_tp_array[target_idx]);
//   RCLCPP_INFO(logger_, "stering tire angle !!!!!!: %f",di);
//   RCLCPP_INFO(logger_, "target index: %d", target_idx);

  AckermannLateralCommand  cmd_msg;
  LateralOutput output;
  cmd_msg.stamp = clock_->now();
  cmd_msg.steering_tire_angle=di;
  output.control_cmd = cmd_msg;
//   output.sync_data.is_steer_converged = calcIsSteerConverged(cmd_msg, current_steering_,converged_steer_rad_);

  
  return output;
}




double CemLateralController::calc_distance(double x_,double y_,double x,double y) 
{
    double dx = x_ - x; // Obliczenie różnicy między x-ami
    double dy = y_ - y; // Obliczenie różnicy między y-ami
    return std::sqrt(dx * dx + dy * dy); 
}

double CemLateralController::yawFromPose(const geometry_msgs::msg::Pose& pose) {

    const auto& quaternion = pose.orientation;

    return std::atan2(2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
                      quaternion.w * quaternion.w + quaternion.x * quaternion.x -
                      quaternion.y * quaternion.y - quaternion.z * quaternion.z);
}


double CemLateralController::cemSteerControl(const geometry_msgs::msg::Pose& state, double Lf, const autoware_auto_planning_msgs::msg::TrajectoryPoint& target_state) {
    const int iter_cem = 15; 
    const int elite_size = 10; 
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
    return delta; 
}

double CemLateralController::getAction(const std::vector<std::tuple<double, double, std::vector<double>>>& cost_tuple_list, int elite_size) {
    std::vector<double> elites;
    // double sigma = 1.0; 

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

    // using State = geometry_msgs::msg::Pose;

    const int iter_cem = 20; 
    const double cost_err = 0.1; 
    // double mu = 0.0; 
    // double sigma = 0.9; 
    const int traj_dim = 30;
    std::vector<double> sim_state;

    std::vector<std::tuple<double, double, std::vector<double> >> cost_tuple_list;

    std::vector<double> x0 = {state.position.x, state.position.y, yaw, v}; // 

    for (int iteration = 0; iteration < iter_cem; ++iteration) {

        double steer_angle = randomSteer(mu, sigma);

        // RCLCPP_INFO(logger_, "ranfom stering: %f", steer_angle);

       
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

// ===================
}  // namespace cem_controller


