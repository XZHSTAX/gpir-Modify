# 权限分配模块 (Authority Allocation Module)

## 概述

权限分配模块为G29共享控制系统提供了灵活的权限分配框架。该模块允许根据不同的策略动态调整人类驾驶员和自动驾驶系统之间的控制权重（alpha值）。

## 文件结构

```
shared_control/
├── src/
│   ├── authority_allocator.py      # 权限分配核心模块
│   └── g29_controller.py           # 修改后的G29控制器（集成权限分配）
├── scripts/
│   └── authority_strategy_switcher.py  # 策略切换工具
├── launch/
│   └── g29_with_authority_allocation.launch  # 启动配置
└── README_Authority_Allocation.md  # 本文档
```

## 核心概念

### Alpha值
- **alpha = 1.0**: 完全人类控制
- **alpha = 0.0**: 完全机器控制  
- **alpha = 0.5**: 人类和机器各占50%权重

### 权限分配策略

#### 1. ConstantAlphaStrategy (固定Alpha策略)
- **用途**: 测试和简单场景
- **特点**: 始终返回固定的alpha值
- **参数**: `alpha_value` - 固定的alpha值

#### 2. SteeringBasedStrategy (基于方向盘的策略)
- **用途**: 根据驾驶员方向盘输入强度调整权限
- **特点**: 方向盘输入越大，人类权限越高
- **参数**: 
  - `min_alpha`: 最小alpha值 (默认0.1)
  - `max_alpha`: 最大alpha值 (默认0.9)
  - `steering_threshold`: 方向盘输入阈值 (默认0.1)

#### 3. EmergencyOverrideStrategy (紧急覆盖策略)
- **用途**: 紧急情况下的权限切换
- **特点**: 检测到紧急情况时立即切换控制权
- **参数**:
  - `emergency_alpha`: 紧急情况下的alpha值 (默认1.0)
  - `normal_alpha`: 正常情况下的alpha值 (默认0.5)

#### 4. AdaptiveStrategy (自适应策略)
- **用途**: 综合多因素的智能权限分配
- **特点**: 考虑驾驶员注意力、方向盘输入、车速、道路曲率等
- **参数**: `base_alpha` - 基础alpha值 (默认0.5)

## 使用方法

### 1. 基本启动

```bash
# 使用默认配置启动
roslaunch shared_control g29_with_authority_allocation.launch

# 指定权限分配策略
roslaunch shared_control g29_with_authority_allocation.launch authority_strategy:=SteeringBased

# 指定初始alpha值（仅对ConstantAlpha策略有效）
roslaunch shared_control g29_with_authority_allocation.launch alpha:=0.7
```

### 2. 运行时策略切换

#### 使用策略切换工具

```bash
# 交互式模式
rosrun shared_control authority_strategy_switcher.py

# 命令行模式
rosrun shared_control authority_strategy_switcher.py switch SteeringBased
rosrun shared_control authority_strategy_switcher.py status
rosrun shared_control authority_strategy_switcher.py monitor 15
```

#### 编程方式切换

```python
# 在G29Controller实例中
controller.switch_authority_strategy('AdaptiveStrategy')

# 获取当前信息
info = controller.get_authority_strategy_info()
print(f"当前策略: {info['current_strategy']}")
print(f"当前Alpha: {info['current_alpha']}")
print(f"可用策略: {info['available_strategies']}")
```

### 3. 监控Alpha值变化

```bash
# 订阅alpha值话题
rostopic echo /shared_control/alpha

# 使用rqt_plot可视化
rqt_plot /shared_control/alpha/data
```

## 自定义权限分配策略

### 1. 创建新策略类

```python
from authority_allocator import AuthorityAllocationStrategy

class MyCustomStrategy(AuthorityAllocationStrategy):
    def __init__(self, custom_param=1.0):
        super().__init__("MyCustom")
        self.custom_param = custom_param
    
    def compute_alpha(self, context):
        # 实现你的权限分配逻辑
        steering = abs(context.get('steering_angle', 0.0))
        # 自定义计算逻辑
        alpha = min(1.0, steering * self.custom_param)
        return alpha
```

### 2. 注册新策略

```python
# 在G29Controller初始化中
custom_strategy = MyCustomStrategy(custom_param=2.0)
self.authority_allocator.register_strategy(custom_strategy)
```

### 3. 上下文信息扩展

在`build_authority_context`方法中添加更多上下文信息：

```python
def build_authority_context(self, human_control):
    context = {
        # 现有上下文...
        'vehicle_speed': self.get_vehicle_speed(),  # 需要实现
        'driver_attention': self.get_driver_attention(),  # 需要实现
        'road_curvature': self.get_road_curvature(),  # 需要实现
        'emergency_detected': self.detect_emergency(),  # 需要实现
    }
    return context
```

## 配置参数

### Launch文件参数

| 参数名 | 类型 | 默认值 | 描述 |
|--------|------|--------|------|
| `ego_vehicle_name` | string | "ego_vehicle" | 车辆名称 |
| `alpha` | float | 0.5 | 初始alpha值 |
| `authority_strategy` | string | "ConstantAlpha" | 权限分配策略 |

### 策略参数

可以通过ROS参数服务器设置策略特定参数：

```xml
<!-- 在launch文件中 -->
<param name="steering_based/min_alpha" value="0.2" />
<param name="steering_based/max_alpha" value="0.8" />
<param name="steering_based/steering_threshold" value="0.15" />
```

## ROS话题和服务

### 发布的话题

- `/shared_control/alpha` (std_msgs/Float64): 当前alpha值
- `/human_control_input` (carla_msgs/CarlaEgoVehicleControl): 人类控制输入
- `/carla/{ego_vehicle_name}/vehicle_control_cmd` (carla_msgs/CarlaEgoVehicleControl): 最终混合控制命令

### 订阅的话题

- `/carla/{ego_vehicle_name}/vehicle_control_cmd_tmp` (carla_msgs/CarlaEgoVehicleControl): 机器控制输入
- `/shared_control/strategy_command` (std_msgs/String): 策略切换命令

## 调试和故障排除

### 1. 启用调试日志

```bash
# 设置日志级别
rosparam set /g29_controller/log_level DEBUG
```

### 2. 常见问题

**问题**: Alpha值不更新
- **解决**: 检查权限分配器是否正确初始化
- **检查**: `rostopic echo /shared_control/alpha`

**问题**: 策略切换失败
- **解决**: 确认策略名称正确，检查可用策略列表
- **检查**: 调用`get_authority_strategy_info()`

**问题**: 控制不平滑
- **解决**: 在自定义策略中添加平滑处理
- **建议**: 使用移动平均或低通滤波

### 3. 性能监控

```bash
# 监控节点性能
rostopic hz /shared_control/alpha
rostopic bw /shared_control/alpha

# 检查计算延迟
rostopic delay /shared_control/alpha
```

## 扩展建议

1. **传感器集成**: 添加眼动追踪、心率监测等驾驶员状态传感器
2. **环境感知**: 集成车道检测、障碍物检测等环境信息
3. **学习能力**: 实现基于驾驶员行为的自适应学习
4. **安全机制**: 添加权限分配的安全约束和故障检测
5. **可视化**: 开发实时权限分配状态的可视化界面

## 许可证

本模块遵循与主项目相同的许可证。