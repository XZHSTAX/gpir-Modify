/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include <ackermann_msgs/AckermannDrive.h>

#include "common/base/state.h"
#include "common/base/trajectory.h"

namespace planning {
namespace simulation {

class PurePursuitController {
 public:
  PurePursuitController();
  ~PurePursuitController() = default;

  void set_wheel_base(const double wheel_base) { wheel_base_ = wheel_base; }

  /**
   * @brief Compute the control command for the vehicle
   *
   * @param state current state of the vehicle
   * @param trajectory reference trajectory
   * @param cmd control command to be filled
   * @return true if the control command is computed successfully
   * @return false otherwise
   */
  bool CalculateAckermannDrive(const common::State& state,
                               const common::Trajectory& trajectory,
                               ackermann_msgs::AckermannDrive* cmd);

 private:
  double wheel_base_ = 2.85;
  double lookahead_distance_ = 15.0;
  double last_delta_ = 0.0;
  double filter_weight_ = 0.2;
};

}  // namespace simulation
}  // namespace planning