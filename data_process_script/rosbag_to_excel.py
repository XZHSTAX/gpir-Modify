#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROSBag数据提取脚本
从指定的rosbag文件中提取数据并转换为Excel文件

作者: AI Assistant
日期: 2025-06-16
"""

import os
import sys
import rosbag
import pandas as pd
import numpy as np
from tf.transformations import euler_from_quaternion
from collections import defaultdict
import argparse


def extract_control_data(msg):
    """
    提取控制消息数据 (throttle, steer, brake)
    
    Args:
        msg: 控制消息
        
    Returns:
        dict: 包含throttle, steer, brake的字典
    """
    return {
        'throttle': msg.throttle,
        'steer': msg.steer,
        'brake': msg.brake,
        'timestamp': msg.header.stamp.to_sec()
    }


def extract_vehicle_status_data(vehicle_status_msg, odometry_msg=None):
    """
    提取车辆状态数据
    
    Args:
        vehicle_status_msg: CarlaEgoVehicleStatus消息
        odometry_msg: nav_msgs/Odometry消息
        
    Returns:
        dict: 包含处理后的车辆状态数据
    """
    data = {'velocity': vehicle_status_msg.velocity}
    
    # 提取加速度 (世界坐标系)
    acc_x = vehicle_status_msg.acceleration.linear.x
    acc_y = vehicle_status_msg.acceleration.linear.y
    acc_z = vehicle_status_msg.acceleration.linear.z
    
    # 提取四元数并计算航向角
    orientation = vehicle_status_msg.orientation
    quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
    roll, pitch, yaw = euler_from_quaternion(quaternion)
    data['yaw'] = yaw
    
    # 计算车身坐标系下的加速度
    # 将世界坐标系加速度转换到车身坐标系
    cos_yaw = np.cos(yaw)
    sin_yaw = np.sin(yaw)
    
    # 车身坐标系: x轴向前(纵向), y轴向左(横向)
    a_lon = acc_x * cos_yaw + acc_y * sin_yaw  # 纵向加速度
    a_lat = -acc_x * sin_yaw + acc_y * cos_yaw  # 横向加速度
    
    data['a_lat'] = a_lat
    data['a_lon'] = a_lon
    
    # 如果有里程计数据，提取位置和速度信息
    if odometry_msg:
        data['x'] = odometry_msg.pose.pose.position.x
        data['y'] = odometry_msg.pose.pose.position.y
        
        # 提取车身坐标系下的速度
        v_x = odometry_msg.twist.twist.linear.x
        v_y = odometry_msg.twist.twist.linear.y
        
        # 转换到车身坐标系
        v_lon = v_x * cos_yaw + v_y * sin_yaw  # 纵向速度
        v_lat = -v_x * sin_yaw + v_y * cos_yaw  # 横向速度
        
        data['v_lat'] = v_lat
        data['v_lon'] = v_lon
        
        # 提取横摆角速度 (yaw rate)
        data['dot_yaw'] = odometry_msg.twist.twist.angular.z
    
    return data


def extract_objects_data(msg):
    """
    提取其他车辆数据
    
    Args:
        msg: derived_object_msgs/ObjectArray消息
        
    Returns:
        dict: 以车辆ID为键的字典，包含每个车辆的状态信息
    """
    objects_data = {}
    
    for obj in msg.objects:
        obj_id = obj.id
        
        # 提取位置
        x = obj.pose.position.x
        y = obj.pose.position.y
        
        # 提取速度
        v_x = obj.twist.linear.x
        v_y = obj.twist.linear.y
        v = np.sqrt(v_x**2 + v_y**2)
        
        # 提取加速度
        a_x = obj.accel.linear.x
        a_y = obj.accel.linear.y
        a = np.sqrt(a_x**2 + a_y**2)
        
        # 提取航向角
        orientation = obj.pose.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(quaternion)
        
        objects_data[f'vehicle_{obj_id}'] = {
            'x': x,
            'y': y,
            'v': v,
            'a': a,
            'yaw': yaw
        }
    
    return objects_data


def extract_joy_data(msg):
    """
    提取手柄数据，将按钮转换为十进制
    
    Args:
        msg: sensor_msgs/Joy消息
        
    Returns:
        dict: 包含按钮十进制值的字典
    """
    # 将11个按钮状态转换为十进制数
    buttons = msg.buttons[:11]  # 确保只取前11个按钮
    decimal_value = 0
    for i, button in enumerate(buttons):
        if button:
            decimal_value += 2**i
    
    return {'buttons_decimal': decimal_value}


def process_rosbag(bag_path, output_path):
    """
    处理单个rosbag文件
    
    Args:
        bag_path (str): rosbag文件路径
        output_path (str): 输出Excel文件路径
    """
    print(f"正在处理: {bag_path}")
    
    # 定义话题映射
    topic_mapping = {
        '/human_control_input': 'human_input',
        '/carla/hero/vehicle_control_cmd': 'final_input',
        '/carla/hero/vehicle_control_cmd_machine': 'machine_input',
        '/carla/hero/vehicle_status': 'ego_vehicle_status',
        '/carla/hero/odometry': 'ego_vehicle_status',  # 合并到同一个sheet
        '/carla/objects': 'other_vehicles_status',
        '/joy': 'joy',
        'external_torque': 'external_torque',
        'machine_torque': 'machine_torque',
        'move_base_simple/goal': 'start_point',
        '/shared_control/strategy_command': 'trans_point',
        '/shared_control/alpha': 'alpha'
    }
    
    # 存储每个sheet的数据
    sheet_data = defaultdict(list)
    
    # 存储车辆状态和里程计数据，用于合并
    vehicle_status_data = {}
    odometry_data = {}
    
    try:
        with rosbag.Bag(bag_path, 'r') as bag:
            for topic, msg, t in bag.read_messages():
                timestamp = t.to_sec()
                
                if topic in ['/human_control_input', '/carla/hero/vehicle_control_cmd', '/carla/hero/vehicle_control_cmd_machine']:
                    # 控制输入数据
                    data = extract_control_data(msg)
                    # data['timestamp'] = timestamp
                    sheet_data[topic_mapping[topic]].append(data)
                
                elif topic == '/carla/hero/vehicle_status':
                    # 车辆状态数据
                    vehicle_status_data[timestamp] = msg
                
                elif topic == '/carla/hero/odometry':
                    # 里程计数据
                    odometry_data[timestamp] = msg
                
                elif topic == '/carla/objects':
                    # 其他车辆数据
                    objects_data = extract_objects_data(msg)
                    row_data = {'timestamp': timestamp}
                    row_data.update(objects_data)
                    sheet_data[topic_mapping[topic]].append(row_data)
                
                elif topic == '/joy':
                    # 手柄数据
                    data = extract_joy_data(msg)
                    data['timestamp'] = timestamp
                    sheet_data[topic_mapping[topic]].append(data)
                
                elif topic in ['external_torque', 'machine_torque']:
                    # 力矩数据
                    data = {'value': msg.data, 'timestamp': timestamp}
                    sheet_data[topic_mapping[topic]].append(data)
                
                elif topic == 'move_base_simple/goal':
                    # 起始点数据
                    data = {'header_stamp': msg.header.stamp.to_sec(), 'timestamp': timestamp}
                    sheet_data[topic_mapping[topic]].append(data)
                
                elif topic == '/shared_control/strategy_command':
                    # 策略命令数据
                    data = {'command': msg.data, 'timestamp': timestamp}
                    sheet_data[topic_mapping[topic]].append(data)
                
                elif topic == '/shared_control/alpha':
                    # Alpha数据
                    data = {'alpha': msg.data, 'timestamp': timestamp}
                    sheet_data[topic_mapping[topic]].append(data)
        
        # 处理车辆状态和里程计数据的合并
        for timestamp in vehicle_status_data:
            vehicle_msg = vehicle_status_data[timestamp]
            odometry_msg = odometry_data.get(timestamp, None)
            
            # 如果没有完全匹配的时间戳，寻找最近的里程计数据
            if odometry_msg is None and odometry_data:
                closest_time = min(odometry_data.keys(), key=lambda x: abs(x - timestamp))
                if abs(closest_time - timestamp) < 0.1:  # 100ms容差
                    odometry_msg = odometry_data[closest_time]
            
            data = extract_vehicle_status_data(vehicle_msg, odometry_msg)
            data['timestamp'] = timestamp
            sheet_data['ego_vehicle_status'].append(data)
        
        # 创建Excel文件
        with pd.ExcelWriter(output_path, engine='openpyxl') as writer:
            for sheet_name, data_list in sheet_data.items():
                if data_list:
                    df = pd.DataFrame(data_list)
                    
                    # 处理重复时间戳问题
                    if 'timestamp' in df.columns and len(df) > 0:
                        # 检查是否存在重复时间戳
                        duplicate_timestamps = df['timestamp'].duplicated().any()
                        
                        if duplicate_timestamps:
                            print(f"  发现重复时间戳在sheet '{sheet_name}'中，正在处理...")
                            
                            # 按时间戳分组，对于重复的时间戳进行聚合
                            numeric_cols = df.select_dtypes(include=[np.number]).columns.tolist()
                            non_numeric_cols = [col for col in df.columns if col not in numeric_cols and col != 'timestamp']
                            
                            # 分别处理数值列和非数值列
                            agg_dict = {}
                            for col in numeric_cols:
                                if col != 'timestamp':
                                    agg_dict[col] = 'mean'  # 数值列取平均值
                            
                            for col in non_numeric_cols:
                                agg_dict[col] = 'last'  # 非数值列取最后一个值
                            
                            if agg_dict:  # 如果有需要聚合的列
                                df = df.groupby('timestamp').agg(agg_dict).reset_index()
                            else:  # 如果只有timestamp列，去重即可
                                df = df.drop_duplicates(subset=['timestamp']).reset_index(drop=True)
                            
                            print(f"  处理完成，从 {len(data_list)} 行减少到 {len(df)} 行")
                        
                        # 将timestamp列移到第一列
                        cols = ['timestamp'] + [col for col in df.columns if col != 'timestamp']
                        df = df[cols]
                        
                        # 按时间戳排序
                        df = df.sort_values('timestamp').reset_index(drop=True)
                    
                    df.to_excel(writer, sheet_name=sheet_name, index=False)
        
        print(f"成功生成: {output_path}")
        
    except Exception as e:
        print(f"处理文件 {bag_path} 时出错: {str(e)}")


def main():
    """
    主函数
    """
    parser = argparse.ArgumentParser(description='ROSBag数据提取工具')
    parser.add_argument('--input_dir', default='/home/xzh2/ros1/gpir_Modify/rosrecord/Exp1/Exp1-main', 
                       help='输入目录路径 (默认: rosrecord/Exp1-main)')
    parser.add_argument('--output_dir', default='/home/xzh2/ros1/gpir_Modify/rosrecord/Exp1/Exp1-main/result-mid', 
                       help='输出目录路径 (默认: rosrecord/Exp1-main)')
    
    args = parser.parse_args()
    
    input_dir = args.input_dir
    output_dir = args.output_dir
    
    # 确保输出目录存在
    os.makedirs(output_dir, exist_ok=True)
    
    # 处理目录下的所有.bag文件
    for filename in os.listdir(input_dir):
        if filename.endswith('.bag'):
            bag_path = os.path.join(input_dir, filename)
            output_filename = filename.replace('.bag', '.xlsx')
            output_path = os.path.join(output_dir, output_filename)
            
            process_rosbag(bag_path, output_path)
    
    print("所有文件处理完成！")


if __name__ == '__main__':
    main()