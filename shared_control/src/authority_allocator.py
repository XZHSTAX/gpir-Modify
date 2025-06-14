#!/usr/bin/env python3

import rospy
import numpy as np
from abc import ABC, abstractmethod
from typing import Dict, Any, Optional
from carla_msgs.msg import CarlaEgoVehicleControl
from std_msgs.msg import Float64, String


class AuthorityAllocationStrategy(ABC):
    """权限分配策略抽象基类
    
    定义了权限分配策略的接口，所有具体的权限分配策略都应该继承此类
    """
    
    def __init__(self, name: str):
        """初始化权限分配策略
        
        Args:
            name (str): 策略名称
        """
        self.name = name
        self.last_update_time = rospy.Time.now()
    
    @abstractmethod
    def compute_alpha(self, context: Dict[str, Any]) -> float:
        """计算权限分配系数alpha
        
        Args:
            context (Dict[str, Any]): 包含计算alpha所需的上下文信息
                可能包含的键值对：
                - 'human_control': CarlaEgoVehicleControl - 人类控制输入
                - 'machine_control': CarlaEgoVehicleControl - 机器控制输入
                - 'steering_angle': float - 方向盘角度
                - 'vehicle_speed': float - 车辆速度
                - 'time_delta': float - 时间间隔
                - 'emergency_detected': bool - 是否检测到紧急情况
                - 'driver_attention': float - 驾驶员注意力水平
                - 'road_curvature': float - 道路曲率
                - 'traffic_density': float - 交通密度
                
        Returns:
            float: alpha值 (0.0-1.0)，表示人类控制权重
        """
        pass
    
    def reset(self):
        """重置策略状态
        
        在策略切换或系统重启时调用
        """
        self.last_update_time = rospy.Time.now()


class ConstantAlphaStrategy(AuthorityAllocationStrategy):
    """固定alpha值策略
    
    始终返回固定的alpha值，适用于测试或简单场景
    """
    
    def __init__(self, alpha_value: float = 0.5):
        """初始化固定alpha策略
        
        Args:
            alpha_value (float): 固定的alpha值 (0.0-1.0)
        """
        super().__init__("ConstantAlpha")
        self.alpha_value = max(0.0, min(1.0, alpha_value))
    
    def compute_alpha(self, context: Dict[str, Any]) -> float:
        """返回固定的alpha值
        
        Args:
            context (Dict[str, Any]): 上下文信息（此策略中未使用）
            
        Returns:
            float: 固定的alpha值
        """
        return self.alpha_value


class SteeringBasedStrategy(AuthorityAllocationStrategy):
    """基于方向盘输入的权限分配策略
    
    根据人类驾驶员的方向盘输入强度动态调整权限分配
    方向盘输入越大，人类权限越高
    """
    
    def __init__(self, min_alpha: float = 0.1, max_alpha: float = 0.9, 
                 steering_threshold: float = 0.1):
        """初始化基于方向盘的策略
        
        Args:
            min_alpha (float): 最小alpha值
            max_alpha (float): 最大alpha值
            steering_threshold (float): 方向盘输入阈值
        """
        super().__init__("SteeringBased")
        self.min_alpha = min_alpha
        self.max_alpha = max_alpha
        self.steering_threshold = steering_threshold
    
    def compute_alpha(self, context: Dict[str, Any]) -> float:
        """基于方向盘输入计算alpha值
        
        Args:
            context (Dict[str, Any]): 包含'steering_angle'的上下文
            
        Returns:
            float: 计算得到的alpha值
        """
        steering_angle = abs(context.get('steering_angle', 0.0))
        
        if steering_angle < self.steering_threshold:
            return self.min_alpha
        
        # 线性映射：方向盘角度越大，人类权限越高
        normalized_steering = min(steering_angle / 1.0, 1.0)  # 假设最大方向盘角度为1.0
        alpha = self.min_alpha + (self.max_alpha - self.min_alpha) * normalized_steering
        
        return max(self.min_alpha, min(self.max_alpha, alpha))


class EmergencyOverrideStrategy(AuthorityAllocationStrategy):
    """紧急情况覆盖策略
    
    在检测到紧急情况时，立即将控制权交给人类或机器
    """
    
    def __init__(self, emergency_alpha: float = 1.0, normal_alpha: float = 0.5):
        """初始化紧急覆盖策略
        
        Args:
            emergency_alpha (float): 紧急情况下的alpha值
            normal_alpha (float): 正常情况下的alpha值
        """
        super().__init__("EmergencyOverride")
        self.emergency_alpha = emergency_alpha
        self.normal_alpha = normal_alpha
    
    def compute_alpha(self, context: Dict[str, Any]) -> float:
        """基于紧急情况检测计算alpha值
        
        Args:
            context (Dict[str, Any]): 包含'emergency_detected'的上下文
            
        Returns:
            float: 计算得到的alpha值
        """
        emergency_detected = context.get('emergency_detected', False)
        
        if emergency_detected:
            rospy.logwarn("Emergency detected! Switching to emergency alpha.")
            return self.emergency_alpha
        
        return self.normal_alpha


class AdaptiveStrategy(AuthorityAllocationStrategy):
    """自适应权限分配策略
    
    综合考虑多个因素进行权限分配，包括驾驶员注意力、道路条件、车辆状态等
    """
    
    def __init__(self, base_alpha: float = 0.5):
        """初始化自适应策略
        
        Args:
            base_alpha (float): 基础alpha值
        """
        super().__init__("Adaptive")
        self.base_alpha = base_alpha
        self.alpha_history = []
        self.max_history_length = 10
    
    def compute_alpha(self, context: Dict[str, Any]) -> float:
        """基于多因素自适应计算alpha值
        
        Args:
            context (Dict[str, Any]): 包含多种因素的上下文
            
        Returns:
            float: 计算得到的alpha值
        """
        alpha = self.base_alpha
        
        # 因子1: 驾驶员注意力水平
        driver_attention = context.get('driver_attention', 1.0)
        attention_factor = driver_attention  # 注意力高则增加人类权限
        
        # 因子2: 方向盘输入强度
        steering_angle = abs(context.get('steering_angle', 0.0))
        steering_factor = min(steering_angle * 2.0, 1.0)  # 方向盘输入大则增加人类权限
        
        # 因子3: 车辆速度（高速时减少人类权限）
        vehicle_speed = context.get('vehicle_speed', 0.0)
        speed_factor = max(0.2, 1.0 - vehicle_speed / 30.0)  # 假设30m/s为高速
        
        # 因子4: 道路曲率（弯道时增加人类权限）
        road_curvature = abs(context.get('road_curvature', 0.0))
        curvature_factor = 1.0 + road_curvature * 0.5
        
        # 综合计算
        alpha = self.base_alpha * attention_factor * steering_factor * speed_factor * curvature_factor
        alpha = max(0.0, min(1.0, alpha))
        
        # 平滑处理
        self.alpha_history.append(alpha)
        if len(self.alpha_history) > self.max_history_length:
            self.alpha_history.pop(0)
        
        smoothed_alpha = np.mean(self.alpha_history)
        
        return smoothed_alpha


class AuthorityAllocator:
    """权限分配器
    
    管理不同的权限分配策略，提供统一的接口来计算和更新alpha值
    """
    
    def __init__(self, initial_strategy: Optional[AuthorityAllocationStrategy] = None):
        """初始化权限分配器
        
        Args:
            initial_strategy (AuthorityAllocationStrategy, optional): 初始策略
        """
        self.current_strategy = initial_strategy or ConstantAlphaStrategy(0.5)
        self.available_strategies = {}
        self.current_alpha = 0.5
        
        # ROS发布器，用于发布当前alpha值
        self.alpha_pub = rospy.Publisher(
            "/shared_control/alpha", Float64, queue_size=10
        )
        
        # ROS订阅器，用于接收策略切换命令
        self.strategy_sub = rospy.Subscriber(
            "/shared_control/strategy_command", String, self.strategy_command_callback, queue_size=10
        )
        
        # 注册默认策略
        self.register_strategy(ConstantAlphaStrategy(0.5))
        self.register_strategy(SteeringBasedStrategy())
        self.register_strategy(EmergencyOverrideStrategy())
        self.register_strategy(AdaptiveStrategy())
        
        rospy.loginfo(f"Authority Allocator initialized with strategy: {self.current_strategy.name}")
    
    def register_strategy(self, strategy: AuthorityAllocationStrategy):
        """注册权限分配策略
        
        Args:
            strategy (AuthorityAllocationStrategy): 要注册的策略
        """
        self.available_strategies[strategy.name] = strategy
        rospy.logdebug(f"Registered strategy: {strategy.name}")
    
    def switch_strategy(self, strategy_name: str) -> bool:
        """切换权限分配策略
        
        Args:
            strategy_name (str): 策略名称
            
        Returns:
            bool: 切换是否成功
        """
        if strategy_name in self.available_strategies:
            old_strategy = self.current_strategy.name
            self.current_strategy = self.available_strategies[strategy_name]
            self.current_strategy.reset()
            rospy.loginfo(f"Switched strategy from {old_strategy} to {strategy_name}")
            return True
        else:
            rospy.logwarn(f"Strategy '{strategy_name}' not found. Available: {list(self.available_strategies.keys())}")
            return False
    
    def update_alpha(self, context: Dict[str, Any]) -> float:
        """更新并返回当前的alpha值
        
        Args:
            context (Dict[str, Any]): 计算alpha所需的上下文信息
            
        Returns:
            float: 更新后的alpha值
        """
        try:
            self.current_alpha = self.current_strategy.compute_alpha(context)
            
            # 发布alpha值
            alpha_msg = Float64()
            alpha_msg.data = self.current_alpha
            self.alpha_pub.publish(alpha_msg)
            
            rospy.logdebug(f"Alpha updated to {self.current_alpha:.3f} using {self.current_strategy.name}")
            
        except Exception as e:
            rospy.logerr(f"Error computing alpha with strategy {self.current_strategy.name}: {e}")
            # 使用默认值
            self.current_alpha = 0.5
        
        return self.current_alpha
    
    def get_current_alpha(self) -> float:
        """获取当前的alpha值
        
        Returns:
            float: 当前的alpha值
        """
        return self.current_alpha
    
    def get_current_strategy_name(self) -> str:
        """获取当前策略名称
        
        Returns:
            str: 当前策略名称
        """
        return self.current_strategy.name
    
    def get_available_strategies(self) -> list:
        """获取可用策略列表
        
        Returns:
            list: 可用策略名称列表
        """
        return list(self.available_strategies.keys())
    
    def strategy_command_callback(self, msg: String):
        """策略切换命令回调函数
        
        Args:
            msg (String): 包含策略名称的ROS消息
        """
        strategy_name = msg.data.strip()
        
        if strategy_name:
            success = self.switch_strategy(strategy_name)
            if success:
                rospy.loginfo(f"Successfully switched to strategy: {strategy_name}")
            else:
                rospy.logwarn(f"Failed to switch to strategy: {strategy_name}. Available strategies: {self.get_available_strategies()}")
        else:
            rospy.logwarn("Received empty strategy command")