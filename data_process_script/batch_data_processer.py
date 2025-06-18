# -*- coding: utf-8 -*-
"""
批量数据处理脚本
读取指定文件夹下的所有.xlsx文件，提取指标数据并按驾驶员分组汇总到新的Excel文件中
"""

import pandas as pd
import os
import re
from collections import defaultdict
import TwoDimTTC

def extract_driver_id(filename):
    """
    从文件名中提取驾驶员ID
    
    Args:
        filename (str): 文件名，如 'Exp1_D1_2025-06-17-22-00-20.xlsx'
    
    Returns:
        str: 驾驶员ID，如 'D1'
    """
    match = re.search(r'_D(\d+)_', filename)
    if match:
        return f"D{match.group(1)}"
    return None

def calu_TTC(ego_vehicle_status,other_vehicle_status):
    """
    计算TTC指标
    """
    # 提取x,y,v_x,v_y,h_x,h_y
    ego_info = ego_vehicle_status[['x','y','v_x','v_y','h_x','h_y']].copy()
    other_info = other_vehicle_status[['x','y','v_x','v_y','h_x','h_y']].copy()

    ego_info.rename(columns={
        'x':'x_i',
        'y':'y_i',
        'v_x':'vx_i',
        'v_y':'vy_i',
        'h_x':'hx_i',
        'h_y':'hy_i'
    }, inplace=True)
    ego_info['length_i'] = 4.69
    ego_info['width_i'] = 1.85

    other_info.rename(columns={
        'x':'x_j',
        'y':'y_j',
        'v_x':'vx_j',
        'v_y':'vy_j',
        'h_x':'hx_j',
        'h_y':'hy_j'
    }, inplace=True)
    other_info['length_j'] = 4.925
    other_info['width_j'] = 1.86
    combined_df = pd.concat([ego_info, other_info], axis=1)

    TTC = TwoDimTTC.TTC(combined_df,'values')

    return TTC



def process_single_file(file_path):
    """
    处理单个Excel文件，提取指标数据
    
    Args:
        file_path (str): Excel文件路径
    
    Returns:
        dict: 包含各项指标的字典
    """
    try:
        # 读取ego_vehicle_status sheet
        ego_vehicle_status = pd.read_excel(file_path, sheet_name="ego_vehicle_status")
        
        # 计算各项指标
        metrics = {
            'a_lon_max': ego_vehicle_status['a_lon'].abs().max(),
            'a_lat_max': ego_vehicle_status['a_lat'].abs().max(),
            'dot_yaw_max': ego_vehicle_status['dot_yaw'].abs().max(),
            'dot_yaw_ave': ego_vehicle_status['dot_yaw'].abs().mean()
        }

        # 读取excel文件，并且获得所有的sheet名称
        excel_file = pd.ExcelFile(file_path)
        sheet_names = excel_file.sheet_names

        # 筛选sheet_names名称中以vehicle开头的，其中后续数字最小的是自车，其他的是其他车辆
        vehicle_sheet_names = [name for name in sheet_names if name.startswith('vehicle')]
        vehicle_sheet_names.sort(key=lambda x: int(x.split('_')[-1]))
        ego_sheet_name = vehicle_sheet_names[0]
        other_sheet_names = vehicle_sheet_names[1:]

        # 读取自车和其他车辆的sheet
        ego_vehicle_status = pd.read_excel(file_path, sheet_name=ego_sheet_name)
        # 遍历其他车辆，计算得到最小TTC
        min_TTC = float('inf')
        for other_sheet_name in other_sheet_names:
            other_vehicle_status = pd.read_excel(file_path, sheet_name=other_sheet_name)
            TTC = calu_TTC(ego_vehicle_status,other_vehicle_status)
            if TTC.min() < min_TTC:
                min_TTC = TTC.min()
        metrics['min_TTC'] = min_TTC

        
        return metrics
    except Exception as e:
        print(f"处理文件 {file_path} 时出错: {e}")
        return None

def main():
    """
    主函数：批量处理所有Excel文件并生成汇总报告
    """
    # 设置输入和输出路径
    input_dir = "rosrecord/Exp1/Exp1-main/result/"
    output_file = "batch_analysis_results.xlsx"
    
    # 检查输入目录是否存在
    if not os.path.exists(input_dir):
        print(f"错误：输入目录 {input_dir} 不存在")
        return
    
    # 获取所有xlsx文件
    xlsx_files = [f for f in os.listdir(input_dir) if f.endswith('.xlsx')]
    
    if not xlsx_files:
        print(f"在目录 {input_dir} 中未找到xlsx文件")
        return
    
    print(f"找到 {len(xlsx_files)} 个xlsx文件")
    
    # 按驾驶员分组存储数据
    driver_data = defaultdict(list)
    
    # 处理每个文件
    for filename in xlsx_files:
        file_path = os.path.join(input_dir, filename)
        driver_id = extract_driver_id(filename)
        
        if driver_id is None:
            print(f"警告：无法从文件名 {filename} 中提取驾驶员ID")
            continue
        
        print(f"正在处理: {filename} (驾驶员: {driver_id})")
        
        metrics = process_single_file(file_path)
        if metrics is not None:
            driver_data[driver_id].append(metrics)
    
    # 检查是否有有效数据
    if not driver_data:
        print("错误：未能处理任何有效数据")
        return
    
    # 创建Excel写入器
    with pd.ExcelWriter(output_file, engine='openpyxl') as writer:
        
        # 获取所有指标名称
        all_metrics = set()
        for driver_metrics_list in driver_data.values():
            for metrics in driver_metrics_list:
                all_metrics.update(metrics.keys())
        
        # 为每个指标创建一个sheet
        for metric in sorted(all_metrics):
            # 准备数据
            data = {'序号': []}
            
            # 获取所有驾驶员ID并排序
            driver_ids = sorted(driver_data.keys())
            
            # 计算最大实验次数
            max_experiments = max(len(driver_data[driver_id]) for driver_id in driver_ids)
            
            # 创建序号列
            data['序号'] = list(range(1, max_experiments + 1))
            
            # 为每个驾驶员创建列
            for driver_id in driver_ids:
                column_name = f"Driver{driver_id[1:]}"
                driver_metrics = driver_data[driver_id]
                
                # 提取该驾驶员的指标值
                values = []
                for metrics in driver_metrics:
                    values.append(metrics.get(metric, None))
                
                # 补齐到最大实验次数
                while len(values) < max_experiments:
                    values.append(None)
                
                data[column_name] = values
            
            # 创建DataFrame并写入sheet
            df = pd.DataFrame(data)
            df.to_excel(writer, sheet_name=metric, index=False)
            
            print(f"已创建sheet: {metric}")
    
    print(f"\n数据处理完成！结果已保存到: {output_file}")
    
    # 打印统计信息
    print("\n统计信息:")
    for driver_id in sorted(driver_data.keys()):
        print(f"  {driver_id}: {len(driver_data[driver_id])} 次实验")

if __name__ == "__main__":
    main()