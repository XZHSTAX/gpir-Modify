# ROSBag数据提取工具

这个Python脚本用于从ROS bag文件中提取指定话题的数据，并将其转换为Excel格式进行分析。

## 功能特性

- 批量处理指定目录下的所有.bag文件
- 提取多种ROS话题数据
- 自动进行坐标系转换和数据计算
- 输出格式化的Excel文件，每个话题对应一个sheet

## 支持的话题

| 话题名称 | Sheet名称 | 提取内容 |
|---------|-----------|----------|
| `/human_control_input` | human_input | throttle, steer, brake |
| `/carla/hero/vehicle_control_cmd` | final_input | throttle, steer, brake |
| `/carla/hero/vehicle_control_cmd_tmp` | machine_input | throttle, steer, brake |
| `/carla/hero/vehicle_status` + `/carla/hero/odometry` | ego_vehicle_status | velocity, yaw, a_lat, a_lon, x, y, v_lat, v_lon, dot_yaw |
| `/carla/objects` | other_vehicles_status | 每个车辆ID的x, y, v, a, yaw |
| `/joy` | joy | 按钮状态的十进制值 |
| `external_torque` | external_torque | 力矩值 |
| `machine_torque` | machine_torque | 力矩值 |
| `move_base_simple/goal` | start_point | header.stamp |
| `/shared_control/strategy_command` | trans_point | 策略命令字符串 |
| `/shared_control/alpha` | alpha | alpha值 |

## 安装依赖

```bash
pip install -r requirements.txt
```

## 使用方法

### 基本用法

```bash
python rosbag_to_excel.py
```

默认处理 `rosrecord/Exp1-main` 目录下的所有.bag文件，输出到同一目录。

### 指定输入和输出目录

```bash
python rosbag_to_excel.py --input_dir /path/to/bag/files --output_dir /path/to/output
```

## 数据处理说明

### 车辆状态数据处理

1. **航向角计算**: 从四元数转换为欧拉角得到yaw角
2. **坐标系转换**: 将世界坐标系的加速度转换为车身坐标系
   - 纵向加速度 (a_lon): 车辆前进方向
   - 横向加速度 (a_lat): 车辆左侧方向
3. **速度转换**: 将全局坐标系速度转换为车身坐标系速度
4. **横摆角速度**: 从odometry消息的angular.z获取dot_yaw

### 其他车辆数据处理

- 每个车辆ID作为独立的列
- 计算速度和加速度的模长
- 提取航向角信息

### 手柄数据处理

- 将11个按钮的状态转换为十进制数
- 按钮i的权重为2^i

## 输出格式

- 每个.bag文件生成一个同名的.xlsx文件
- 每个话题对应Excel中的一个sheet
- 第一列始终为时间戳
- 后续列为相应的数据字段

## 注意事项

1. 确保ROS环境已正确配置
2. 车辆状态和里程计数据会自动匹配时间戳（容差100ms）
3. 如果某些话题在bag文件中不存在，对应的sheet将为空
4. 脚本会自动处理坐标系转换，确保数据的一致性

## 错误处理

- 脚本会捕获并报告处理过程中的错误
- 即使某个文件处理失败，也会继续处理其他文件
- 详细的错误信息会输出到控制台

## 示例输出

处理完成后，每个Excel文件将包含以下sheet：
- human_input: 人工控制输入
- final_input: 最终控制输入
- machine_input: 机器控制输入
- ego_vehicle_status: 自车状态
- other_vehicles_status: 其他车辆状态
- joy: 手柄输入
- external_torque: 外部力矩
- machine_torque: 机器力矩
- start_point: 起始点
- trans_point: 切换点
- alpha: Alpha参数