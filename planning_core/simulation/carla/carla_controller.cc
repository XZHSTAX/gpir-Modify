/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include <ackermann_msgs/AckermannDrive.h>
#include <ros/ros.h>

#include <algorithm>

#include "carla_msgs/CarlaEgoVehicleControl.h"
#include "carla_msgs/CarlaEgoVehicleStatus.h"

constexpr double kInf = std::numeric_limits<double>::infinity();

class PIDController {
 public:
  PIDController(const double kp, const double ki, const double kd)
      : kp_(kp), ki_(ki), kd_(kd) {
    previous_error_ = 0.0;
    integral_ = 0.0;
    use_integral = ki_ > 1e-5;
  }

  void SetIntegralLimit(const double max, const double min) {
    max_integral_ = max;
    min_integral_ = min;
  }

  void SetOutputLimit(const double max, const double min) {
    max_output_ = max;
    min_output_ = min;
  }

  double Control(const double error, const double dt) {
    const double diff = (error - previous_error_) / dt;

    if (use_integral) {
      integral_ += error * dt * ki_;
      integral_ = std::max(std::min(integral_, max_integral_), min_integral_);
    }

    previous_error_ = error;
    double output = error * kp_ + integral_ + diff * kd_;
    return std::max(std::min(output, max_output_), min_output_);
  }

  void Reset() {
    integral_ = 0.0;
    previous_error_ = 0.0;
  }

 private:
  double kp_ = 0.0;
  double ki_ = 0.0;
  double kd_ = 0.0;
  double previous_error_ = 0.0;
  bool use_integral = false;

  double integral_ = 0.0;
  double max_integral_ = kInf;
  double min_integral_ = -kInf;
  double max_output_ = kInf;
  double min_output_ = -kInf;
};

double current_speed = 0.0;
double target_speed = 0.0, target_acc = 0.0, target_steer = 0.0;
bool control_started = false;  // 标志变量，表示是否已经开始控制

void TargetCallBack(const ackermann_msgs::AckermannDrive& msg) {
  // 只有在第一次接收到speed > 0的消息时才开始控制
  if (!control_started && msg.speed > 0.0) {
    control_started = true;
    ROS_INFO("Control started: received first valid ackermann command with speed > 0");
  }
  
  target_speed = msg.speed;
  target_acc = msg.acceleration;
  target_steer = msg.steering_angle;
}

void StateCallBack(const carla_msgs::CarlaEgoVehicleStatus& msg) {
  current_speed = msg.velocity;
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "carla_controller");
  ros::NodeHandle node("~");
  double kp, ki, kd;
  std::string ego_vehicle_name;
  
  node.getParam("kp", kp);
  node.getParam("ki", ki);
  node.getParam("kd", kd);
  // 从参数中获取ego_vehicle名称，默认为ego_vehicle
  node.param<std::string>("ego_vehicle_name", ego_vehicle_name, "ego_vehicle");

  ROS_INFO_STREAM("kp: " << kp);
  ROS_INFO_STREAM("ki: " << ki);
  ROS_INFO_STREAM("kd: " << kd);
  ROS_INFO_STREAM("ego_vehicle_name: " << ego_vehicle_name);

  // 构建topic名称
  std::string ackermann_topic = "/carla/" + ego_vehicle_name + "/ackermann_cmd";
  std::string status_topic = "/carla/" + ego_vehicle_name + "/vehicle_status";
  std::string control_topic = "/carla/" + ego_vehicle_name + "/vehicle_control_cmd_tmp";

  ros::Publisher control_pub;
  ros::Subscriber target_sub, state_sub;
  target_sub = node.subscribe(ackermann_topic, 1, &TargetCallBack);
  state_sub = node.subscribe(status_topic, 1, &StateCallBack);
  control_pub = node.advertise<carla_msgs::CarlaEgoVehicleControl>(
      control_topic, 1);

  PIDController speed_controller(kp, ki, kd);
  speed_controller.SetOutputLimit(4.0, -8);
  speed_controller.SetIntegralLimit(20, -15.0);

  ros::Rate rate(50);
  while (ros::ok()) {
    ros::spinOnce();
    
    carla_msgs::CarlaEgoVehicleControl control_cmd;
    control_cmd.header.stamp = ros::Time::now();
    
    // 只有在控制开始后才进行实际的控制计算和发布
    if (control_started) {
      double acc = speed_controller.Control(target_speed - current_speed, 0.02);

      if (acc >= 0) {
        control_cmd.throttle = std::min(acc / 4.0, 1.0);
        control_cmd.brake = 0.0;
      } else {
        control_cmd.throttle = 0.0;
        control_cmd.brake = std::min(std::fabs(acc) / 8.0, 1.0);
      }

      if (target_speed == 0.0) {
        control_cmd.throttle = 0.0;
        control_cmd.brake = 1.0;
        speed_controller.Reset();
      }

      control_cmd.steer = -target_steer / 1.22173035145;
    } else {
      // 控制未开始时，发送零控制命令
      control_cmd.throttle = 0.0;
      control_cmd.brake = 0.0;
      control_cmd.steer = 0.0;
    }
    
    control_pub.publish(control_cmd);
    rate.sleep();
  }

  return 0;
}
