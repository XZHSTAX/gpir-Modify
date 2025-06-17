# ROSBag数据后处理结果说明

## 概述

本文档说明了对`rosrecord/Exp1-main/result-mid`文件夹中提取的ROSBag数据进行后处理的结果。处理后的数据保存在`rosrecord/Exp1-main/result`文件夹中。

## 处理步骤

### 1. 时间戳截断
- **起始时间**: 使用`trans_point` sheet的第一个`timestamp`
- **结束时间**: 使用`alpha` sheet中第一次`alpha=1`的`timestamp`
- **结果**: 所有sheet的数据都被截断到这个时间范围内

### 2. 输入数据合并
- **合并的sheet**: `human_input`、`machine_input`、`final_input`
- **列名重命名**:
  - `human_input`: `throttle` → `h_throttle`, `steer` → `h_steer`, `brake` → `h_brake`
  - `machine_input`: `throttle` → `m_throttle`, `steer` → `m_steer`, `brake` → `m_brake`
  - `final_input`: `throttle` → `f_throttle`, `steer` → `f_steer`, `brake` → `f_brake`
- **新增列**: `Delta_steer = h_steer - m_steer`
- **结果sheet**: `merged_input`

### 3. 车辆状态数据分解
- **原始sheet**: `other_vehicles_status`
- **分解方式**: 将除`timestamp`外的每一列（代表一个车辆）提取为单独的sheet
- **新sheet命名**: 以原列名命名（如`vehicle_277`、`vehicle_281`等）
- **新sheet结构**: 第一列为`timestamp`，后续列为解析后的车辆数据

## 处理后的数据结构

### 保留的原始sheet
- `ego_vehicle_status`: 自车状态数据
- `alpha`: 控制权切换数据
- `trans_point`: 交接点数据
- `joy`: 手柄数据

### 新生成的sheet
- `merged_input`: 合并后的输入数据，包含人工、机器和最终输入
- `vehicle_XXX`: 分解后的其他车辆数据（XXX为车辆ID）

## 数据字段说明

### merged_input sheet
| 字段 | 说明 |
|------|------|
| timestamp | 时间戳 |
| h_throttle | 人工油门输入 |
| h_steer | 人工转向输入 |
| h_brake | 人工刹车输入 |
| m_throttle | 机器油门输入 |
| m_steer | 机器转向输入 |
| m_brake | 机器刹车输入 |
| f_throttle | 最终油门输入 |
| f_steer | 最终转向输入 |
| f_brake | 最终刹车输入 |
| Delta_steer | 人工与机器转向差值 (h_steer - m_steer) |

### vehicle_XXX sheet
| 字段 | 说明 |
|------|------|
| timestamp | 时间戳 |
| x | 车辆X坐标 |
| y | 车辆Y坐标 |
| v | 车辆速度 |
| a | 车辆加速度 |
| yaw | 车辆航向角 |

## 使用的脚本

### 主处理脚本
- **文件**: `process_extracted_data.py`
- **功能**: 执行上述所有处理步骤
- **使用方法**: 
  ```bash
  python3 process_extracted_data.py
  ```

### 验证脚本
- **文件**: `verify_processed_data.py`
- **功能**: 验证处理后数据的结构和完整性
- **使用方法**: 
  ```bash
  python3 verify_processed_data.py
  ```

## 处理结果统计

根据验证结果，所有6个文件都已成功处理：
- `Exp1__2025-06-17-10-48-41.xlsx`
- `Exp1__2025-06-17-10-50-10.xlsx`
- `Exp1__2025-06-17-10-51-05.xlsx`
- `Exp1__2025-06-17-10-51-55.xlsx`
- `Exp1__2025-06-17-10-52-52.xlsx`
- `Exp1__2025-06-17-10-56-31.xlsx`

每个文件都包含：
- 时间戳截断后的数据
- 合并的输入数据（`merged_input`）
- 分解的车辆状态数据（多个`vehicle_XXX` sheet）
- 正确计算的`Delta_steer`列

## 注意事项

1. **时间对齐**: 使用0.01秒的容差进行时间戳对齐
2. **数据完整性**: 处理过程中保留了所有有效数据
3. **错误处理**: 脚本包含完善的错误处理和日志输出
4. **数据验证**: 提供了验证脚本确保处理结果的正确性

## 依赖库

- pandas >= 1.0.0
- numpy >= 1.18.0
- openpyxl >= 3.0.7