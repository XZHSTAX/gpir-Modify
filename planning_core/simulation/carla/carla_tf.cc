/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "carla_tf");
  ros::NodeHandle node;
  ros::NodeHandle private_nh("~");
  geometry_msgs::TransformStamped ego_tf;

  // 从参数服务器获取role_name，默认值为"ego_vehicle"
  std::string role_name;
  private_nh.param<std::string>("role_name", role_name, "ego_vehicle");
  
  // 构建2D frame名称
  std::string ego_2d_frame = role_name + "2d";
  
  ROS_INFO("carla_tf: Using role_name: %s", role_name.c_str());
  ROS_INFO("carla_tf: 2D frame name: %s", ego_2d_frame.c_str());

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);
  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster;

  ros::Rate rate(100);
  while (node.ok()) {
    try {
      ego_tf = tf_buffer.lookupTransform("map", role_name, ros::Time(0));
      geometry_msgs::TransformStamped ego_tf_2d;
      ego_tf_2d.header.frame_id = "map";
      ego_tf_2d.header.stamp = ros::Time::now();
      ego_tf_2d.child_frame_id = "ego_vehicle2d";
      ego_tf_2d.transform = ego_tf.transform;
      ego_tf_2d.transform.translation.z = 0;
      double yaw = tf::getYaw(ego_tf_2d.transform.rotation);
      tf::quaternionTFToMsg(tf::createQuaternionFromYaw(yaw),
                            ego_tf_2d.transform.rotation);
      static_tf_broadcaster.sendTransform(ego_tf_2d);
    } catch (tf2::TransformException& ex) {
    }

    rate.sleep();
  }

  return 0;
}
