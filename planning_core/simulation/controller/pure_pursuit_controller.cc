/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include "planning_core/simulation/controller/pure_pursuit_controller.h"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

#include "common/utils/math.h"

namespace planning {
namespace simulation {

PurePursuitController::PurePursuitController() = default;

bool PurePursuitController::CalculateAckermannDrive(
    const common::State& state, const common::Trajectory& trajectory,
    ackermann_msgs::AckermannDrive* cmd) {
  if (trajectory.empty()) {
    return false;
  }

  // Find the nearest point on the trajectory
  int nearest_index = trajectory.GetNearsetIndex(state.position);
  const auto& nearest_point = trajectory[nearest_index];

  // Find the lookahead point
  double lookahead_dist_sq = lookahead_distance_ * lookahead_distance_;
  int lookahead_index = nearest_index;
  for (size_t i = nearest_index; i < trajectory.size(); ++i) {
    double dist_sq = (trajectory[i].position - state.position).squaredNorm();
    if (dist_sq >= lookahead_dist_sq) {
      lookahead_index = i;
      break;
    }
  }
  const auto& lookahead_point = trajectory[lookahead_index];

  // Calculate alpha
  Eigen::Vector2d vec_to_lookahead = lookahead_point.position - state.position;
  double alpha = std::atan2(vec_to_lookahead.y(), vec_to_lookahead.x()) - state.heading;

  // Calculate steering angle
  double delta = std::atan2(2.0 * wheel_base_ * std::sin(alpha), lookahead_distance_);

  // Limit delta to [-30, 30] degrees
  double max_steer_angle_rad = 30.0 * M_PI / 180.0;
  delta = std::max(-max_steer_angle_rad, std::min(delta, max_steer_angle_rad));

  // Low-pass filter
  delta = filter_weight_ * delta + (1 - filter_weight_) * last_delta_;
  last_delta_ = delta;

  // Map delta to [-1, 1]
  cmd->steering_angle = delta / max_steer_angle_rad;
  return true;
}

}  // namespace simulation
}  // namespace planning