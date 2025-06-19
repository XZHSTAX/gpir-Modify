#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROSBag提取数据的后处理脚本

功能:
1. 以时间戳截断数据
2. 对齐并合并human_input, machine_input, final_input
3. 分解other_vehicles_status数据
4. 保存处理后的数据

作者: AI Assistant
日期: 2025-06-17
"""

import pandas as pd
import numpy as np
import os
import argparse
import ast
from pathlib import Path


def parse_dict_column(df, column_name):
    """
    解析包含字典字符串的列
    
    Args:
        df (pd.DataFrame): 数据框
        column_name (str): 列名
        
    Returns:
        pd.DataFrame: 解析后的数据框
    """
    parsed_data = []
    
    for idx, row in df.iterrows():
        timestamp = row['timestamp']
        dict_str = row[column_name]
        
        if pd.isna(dict_str) or dict_str == '':
            continue
            
        try:
            # 尝试解析字典字符串
            if isinstance(dict_str, str):
                data_dict = ast.literal_eval(dict_str)
            else:
                data_dict = dict_str
                
            # 添加时间戳
            data_dict['timestamp'] = timestamp
            parsed_data.append(data_dict)
            
        except (ValueError, SyntaxError) as e:
            print(f"警告: 无法解析第{idx}行的数据: {dict_str}, 错误: {e}")
            continue
    
    if not parsed_data:
        return pd.DataFrame()
    
    return pd.DataFrame(parsed_data)


def align_timestamps(*dataframes, tolerance=0.01):
    """
    对齐多个数据框的时间戳
    
    Args:
        *dataframes: 多个数据框
        tolerance (float): 时间戳容差 (秒)
        
    Returns:
        pd.DataFrame: 对齐后合并的数据框
    """
    if not dataframes:
        return pd.DataFrame()
    
    # 过滤掉空的数据框
    valid_dfs = [df for df in dataframes if not df.empty and 'timestamp' in df.columns]
    
    if not valid_dfs:
        return pd.DataFrame()
    
    # 获取所有时间戳的并集
    all_timestamps = set()
    for df in valid_dfs:
        all_timestamps.update(df['timestamp'].values)
    
    all_timestamps = sorted(list(all_timestamps))
    
    # 创建结果数据框
    result_data = []
    
    for target_time in all_timestamps:
        row_data = {'timestamp': target_time}
        
        for i, df in enumerate(valid_dfs):
            # 找到最接近的时间戳
            time_diffs = np.abs(df['timestamp'] - target_time)
            
            if len(time_diffs) == 0:
                continue
                
            closest_idx = time_diffs.idxmin()
            
            # 检查索引是否有效
            if closest_idx not in df.index:
                continue
                
            if time_diffs.loc[closest_idx] <= tolerance:
                # 在容差范围内，使用该数据
                closest_row = df.loc[closest_idx]
                for col in df.columns:
                    if col != 'timestamp':
                        row_data[col] = closest_row[col]
        
        result_data.append(row_data)
    
    return pd.DataFrame(result_data)


def process_single_file(input_path, output_path,DURATION=None):
    """
    处理单个Excel文件
    
    Args:
        input_path (str): 输入文件路径
        output_path (str): 输出文件路径
    """
    print(f"正在处理: {input_path}")
    
    # 读取所有sheet
    try:
        excel_data = pd.read_excel(input_path, sheet_name=None)
    except Exception as e:
        print(f"错误: 无法读取文件 {input_path}: {e}")
        return
    
    print(f"  发现 {len(excel_data)} 个sheet: {list(excel_data.keys())}")
    
    # 1. 确定时间范围
    start_time = None
    end_time = None
    
    # 获取trans_point的第一个时间戳作为起始时间
    if 'trans_point' in excel_data and not excel_data['trans_point'].empty:
        trans_point_df = excel_data['trans_point']
        if 'timestamp' in trans_point_df.columns and len(trans_point_df) > 0:
            start_time = trans_point_df['timestamp'].iloc[0]
            print(f"  起始时间 (trans_point第一个时间戳): {start_time}")
    if DURATION is not None:
        end_time = start_time + DURATION
    else:
        # 获取alpha第一次等于1的时间戳作为结束时间
        if 'alpha' in excel_data and not excel_data['alpha'].empty:
            alpha_df = excel_data['alpha']
            if 'alpha' in alpha_df.columns and 'timestamp' in alpha_df.columns:
                # 查找alpha等于1的行
                alpha_1_mask = (alpha_df['alpha'] == 1.0) | (alpha_df['alpha'] == 1)
                alpha_1_rows = alpha_df[alpha_1_mask]
                if not alpha_1_rows.empty:
                    end_time = alpha_1_rows['timestamp'].iloc[0]
                    print(f"  结束时间 (alpha第一次等于1): {end_time}")
                else:
                    print(f"  警告: 在alpha sheet中未找到alpha=1的行")
                    print(f"  alpha列的唯一值: {alpha_df['alpha'].unique()}")
    
    if start_time is None or end_time is None:
        print(f"  警告: 无法确定时间范围 (start_time: {start_time}, end_time: {end_time})")
        print(f"  将处理所有数据")
    else:
        print(f"  时间范围: {start_time} 到 {end_time}")
    
    # 2. 截断所有sheet的数据
    filtered_data = {}
    for sheet_name, df in excel_data.items():
        if df.empty or 'timestamp' not in df.columns:
            filtered_data[sheet_name] = df
            continue
            
        if start_time is not None and end_time is not None:
            mask = (df['timestamp'] >= start_time) & (df['timestamp'] <= end_time)
            filtered_df = df[mask].copy()
            print(f"  {sheet_name}: {len(df)} -> {len(filtered_df)} 行")
        else:
            filtered_df = df.copy()
            print(f"  {sheet_name}: {len(df)} 行 (未截断)")
            
        filtered_data[sheet_name] = filtered_df
    
    # 3. 合并human_input, machine_input, final_input
    input_sheets = ['human_input', 'machine_input', 'final_input']
    input_dfs = []
    
    for sheet_name in input_sheets:
        if sheet_name in filtered_data and not filtered_data[sheet_name].empty:
            df = filtered_data[sheet_name].copy()
            
            # 重命名列
            prefix_map = {
                'human_input': 'h_',
                'machine_input': 'm_',
                'final_input': 'f_'
            }
            prefix = prefix_map[sheet_name]
            
            rename_dict = {}
            for col in ['throttle', 'steer', 'brake']:
                if col in df.columns:
                    rename_dict[col] = f"{prefix}{col}"
            
            df = df.rename(columns=rename_dict)
            input_dfs.append(df)
            print(f"  准备合并 {sheet_name}: {len(df)} 行")
    
    if input_dfs:
        # 对齐时间戳并合并
        merged_input = align_timestamps(*input_dfs)
        
        # 计算Delta_steer
        if 'h_steer' in merged_input.columns and 'm_steer' in merged_input.columns:
            merged_input['Delta_steer'] = merged_input['h_steer'] - merged_input['m_steer']
            print(f"  添加Delta_steer列")
        
        # 将时间戳列移到第一列
        if 'timestamp' in merged_input.columns:
            cols = ['timestamp'] + [col for col in merged_input.columns if col != 'timestamp']
            merged_input = merged_input[cols]
        
        filtered_data['merged_input'] = merged_input
        print(f"  合并后的input数据: {len(merged_input)} 行")
        
        # 移除原始的input sheets
        for sheet_name in input_sheets:
            if sheet_name in filtered_data:
                del filtered_data[sheet_name]
    
    # 4. 分解other_vehicles_status数据
    if 'other_vehicles_status' in filtered_data and not filtered_data['other_vehicles_status'].empty:
        other_vehicles_df = filtered_data['other_vehicles_status']
        print(f"  分解other_vehicles_status数据: {len(other_vehicles_df)} 行")
        
        # 获取除timestamp外的所有列
        vehicle_columns = [col for col in other_vehicles_df.columns if col != 'timestamp']
        print(f"  发现车辆列: {vehicle_columns}")
        
        for col in vehicle_columns:
            # 解析每个车辆的数据
            vehicle_data = parse_dict_column(other_vehicles_df, col)
            
            if not vehicle_data.empty:
                # 将时间戳列移到第一列
                if 'timestamp' in vehicle_data.columns:
                    cols = ['timestamp'] + [c for c in vehicle_data.columns if c != 'timestamp']
                    vehicle_data = vehicle_data[cols]
                
                filtered_data[col] = vehicle_data
                print(f"    {col}: {len(vehicle_data)} 行")
        
        # 移除原始的other_vehicles_status sheet
        del filtered_data['other_vehicles_status']
    
    # 5. 保存处理后的数据
    try:
        with pd.ExcelWriter(output_path, engine='openpyxl') as writer:
            for sheet_name, df in filtered_data.items():
                if not df.empty:
                    df.to_excel(writer, sheet_name=sheet_name, index=False)
                    print(f"  保存sheet '{sheet_name}': {len(df)} 行")
        
        print(f"  成功保存到: {output_path}")
        
    except Exception as e:
        print(f"  错误: 保存文件失败: {e}")


def main():
    """
    主函数
    """
    dir_name = 'Exp2/'
    dir_name2 = 'Exp2-Compare1/'
    parser = argparse.ArgumentParser(description='处理ROSBag提取的Excel数据')
    parser.add_argument('--input_dir', type=str, 
                       default='/home/xzh2/ros1/gpir_Modify/rosrecord/'+dir_name+dir_name2+'result-mid',
                       help='输入目录路径')
    parser.add_argument('--output_dir', type=str,
                       default='/home/xzh2/ros1/gpir_Modify/rosrecord/'+dir_name+dir_name2+'result',
                       help='输出目录路径')
    
    args = parser.parse_args()
    
    input_dir = Path(args.input_dir)
    output_dir = Path(args.output_dir)
    
    # 检查输入目录
    if not input_dir.exists():
        print(f"错误: 输入目录不存在: {input_dir}")
        return
    
    # 创建输出目录
    output_dir.mkdir(parents=True, exist_ok=True)
    print(f"输出目录: {output_dir}")
    
    # 查找所有xlsx文件
    xlsx_files = list(input_dir.glob('*.xlsx'))
    
    if not xlsx_files:
        print(f"错误: 在 {input_dir} 中未找到xlsx文件")
        return
    
    print(f"找到 {len(xlsx_files)} 个xlsx文件")
    
    # 处理每个文件
    for xlsx_file in xlsx_files:
        output_file = output_dir / xlsx_file.name
        if 'Compare1' in dir_name2 :
            if dir_name == 'Exp1/':
                duration = 7
            elif dir_name == 'Exp2/':
                duration = 3
            elif dir_name == 'Exp3/':
                duration = 10
            else:
                print('Traget dir_name is not define in here')
            process_single_file(str(xlsx_file), str(output_file),duration)
        else:
            process_single_file(str(xlsx_file), str(output_file))
        print()
    
    print("所有文件处理完成！")


if __name__ == '__main__':
    main()