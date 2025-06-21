#!/bin/zsh
# -*- coding: utf-8 -*-

# 数据处理流水线脚本
# 依次执行rosbag数据提取、预处理和批量分析
# 
# 使用方法:
#   ./run_data_process.sh --Exp_setting Exp1 --method_setting Compare1
#   或者
#   bash run_data_process.sh --Exp_setting Exp1 --method_setting Compare1

# 设置脚本目录
SCRIPT_DIR="/home/xzh2/ros1/gpir_Modify/src/gpir-Modify/data_process_script"

# 检查Python脚本是否存在
check_file_exists() {
    if [ ! -f "$1" ]; then
        echo "错误: 文件不存在: $1"
        exit 1
    fi
}

# 检查所有Python脚本
check_file_exists "$SCRIPT_DIR/rosbag_to_excel.py"
check_file_exists "$SCRIPT_DIR/process_extracted_data.py"
check_file_exists "$SCRIPT_DIR/batch_data_processer.py"

echo "开始数据处理流水线..."
echo "参数: $@"
echo "${(l:50::=:)}"

# 步骤1: ROSBag数据提取
echo "步骤1: 执行ROSBag数据提取..."
python3 "$SCRIPT_DIR/rosbag_to_excel.py" "$@"
if [ $? -ne 0 ]; then
    echo "错误: rosbag_to_excel.py 执行失败"
    exit 1
fi
echo "步骤1完成"
echo "${(l:30::-:)}"

# 步骤2: 数据预处理
echo "步骤2: 执行数据预处理..."
python3 "$SCRIPT_DIR/process_extracted_data.py" "$@"
if [ $? -ne 0 ]; then
    echo "错误: process_extracted_data.py 执行失败"
    exit 1
fi
echo "步骤2完成"
echo "${(l:30::-:)}"

# 步骤3: 批量数据分析
echo "步骤3: 执行批量数据分析..."
python3 "$SCRIPT_DIR/batch_data_processer.py" "$@"
if [ $? -ne 0 ]; then
    echo "错误: batch_data_processer.py 执行失败"
    exit 1
fi
echo "步骤3完成"
echo "------------------------------"

echo "数据处理流水线全部完成！"
echo "${(l:50::=:)}"