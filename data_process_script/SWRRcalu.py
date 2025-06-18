import numpy as np
from scipy import signal
from typing import Optional, Tuple, List


def steering_wheel_reversal_rate(sw_sequence_deg: np.ndarray, 
                                timestamp: np.ndarray,
                                gap_size_deg: float,
                                lowpass_cutoff_freq_hz: Optional[float] = None,
                                lowpass_filter_order: int = 2,
                                debug: bool = False) -> float:
    """
    计算方向盘转向反转率（Steering Wheel Reversal Rate, SWRR）指标。
    
    这是MATLAB版本metric_ReversalRate_VTEC函数的Python实现，用于评估驾驶员
    认知负荷和视觉任务影响。
    
    Args:
        sw_sequence_deg: 方向盘角度序列（度）
        timestamp: 时间戳数组（秒）
        gap_size_deg: 间隙大小，定义反转的最小角度阈值（度）
        lowpass_cutoff_freq_hz: 低通滤波器截止频率（Hz），None表示不滤波
        lowpass_filter_order: 低通滤波器阶数，默认为2
        debug: 是否显示调试信息和图表
        
    Returns:
        float: 每分钟的方向盘反转次数
        
    Example:
        >>> sw_data = np.array([0, 5, 10, 5, 0, -5, -10, -5, 0])
        >>> timestamps = np.array([0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8])
        >>> swrr = steering_wheel_reversal_rate(sw_data, timestamps, 8.0, 2.0)
        >>> print(f"SWRR: {swrr:.2f} reversals/min")
    """
    
    # 转换为numpy数组并确保为列向量
    sw_sequence_deg = np.array(sw_sequence_deg).flatten()
    timestamp = np.array(timestamp).flatten()
    
    # 验证输入数据长度一致性
    if len(sw_sequence_deg) != len(timestamp):
        raise ValueError(f"方向盘角度序列长度 ({len(sw_sequence_deg)}) 与时间戳长度 ({len(timestamp)}) 不匹配")
    
    # 标准化时间戳，使其从0开始
    t_list = timestamp - timestamp[0]
    sequence_length_s = t_list[-1]  # 计算总时长
    
    # 计算平均采样率用于滤波器设计
    if len(t_list) > 1:
        avg_sample_interval = np.mean(np.diff(t_list))
        sample_rate_hz = 1.0 / avg_sample_interval
    else:
        raise ValueError("时间戳数组长度必须大于1")
    
    # 低通滤波
    if lowpass_cutoff_freq_hz is None:
        sw_sequence_filt = sw_sequence_deg
    else:
        # 使用scipy.signal创建Butterworth滤波器
        nyquist_freq = sample_rate_hz / 2
        normalized_cutoff = lowpass_cutoff_freq_hz / nyquist_freq
        
        # 确保截止频率在有效范围内
        if normalized_cutoff >= 1.0:
            print(f"警告: 截止频率 {lowpass_cutoff_freq_hz} Hz 过高，调整为 {nyquist_freq * 0.9} Hz")
            normalized_cutoff = 0.9
        
        # 创建Butterworth滤波器
        b, a = signal.butter(lowpass_filter_order, normalized_cutoff, btype='low')
        
        # 使用零相位滤波（等效于MATLAB的filtfilt）
        sw_sequence_filt = signal.filtfilt(b, a, sw_sequence_deg)
    
    # 核心算法：反转检测
    swrr = _calculate_reversals(sw_sequence_filt, t_list, gap_size_deg, 
                               sequence_length_s, debug)
    
    return swrr


def _calculate_reversals(sw_sequence_filt: np.ndarray, 
                        t_list: np.ndarray,
                        gap_size_deg: float, 
                        sequence_length_s: float,
                        debug: bool = False) -> float:
    """
    计算方向盘反转的核心算法。
    
    Args:
        sw_sequence_filt: 滤波后的方向盘角度序列
        t_list: 时间戳数组
        gap_size_deg: 间隙大小阈值
        sequence_length_s: 序列总时长（秒）
        debug: 是否显示调试信息
        
    Returns:
        float: 每分钟的反转次数
    """
    
    # 计算导数（差分）
    d = np.diff(sw_sequence_filt)
    d = np.concatenate([[0], d])  # 在开头添加0，保持长度一致
    
    # 找到极值点（符号变化点）
    sign_d = np.sign(d)
    sign_changes = np.abs(np.diff(sign_d)) == 2
    zero_points = (d[:-1] == 0)
    extrema_mask = sign_changes | zero_points
    extrema = np.where(extrema_mask)[0] + 1  # +1是因为diff减少了一个元素
    
    # 确保extrema在有效范围内
    extrema = extrema[extrema < len(sw_sequence_filt)]
    
    if len(extrema) == 0:
        return 0.0
    
    # 检测向上和向下的反转
    upward_reversals = _find_upward_reversals(sw_sequence_filt, extrema, gap_size_deg)
    downward_reversals = _find_upward_reversals(-sw_sequence_filt, extrema, gap_size_deg)
    
    # 合并所有反转
    all_reversals = []
    if len(upward_reversals) > 0:
        all_reversals.extend(upward_reversals)
    if len(downward_reversals) > 0:
        all_reversals.extend(downward_reversals)
    
    n_reversals = len(all_reversals)
    
    # 计算每分钟的反转率
    swrr = n_reversals / sequence_length_s * 60
    
    # 调试输出
    if debug:
        print(f"\n*** 方向盘反转率计算结果 ***")
        print(f"序列长度: {sequence_length_s:.2f} 秒")
        print(f"间隙大小: {gap_size_deg} 度")
        print(f"检测到的极值点数量: {len(extrema)}")
        print(f"向上反转数量: {len(upward_reversals)}")
        print(f"向下反转数量: {len(downward_reversals)}")
        print(f"总反转数量: {n_reversals}")
        print(f"SWRR: {swrr:.2f} 反转/分钟")
        
        # 输出调试信息
        print(f"找到 {len(extrema)} 个极值点")
        print(f"找到 {len(all_reversals)} 个反转")
        print(f"SWRR: {swrr:.4f} reversals/min")
    
    return swrr


def _find_upward_reversals(steer_angle_deg: np.ndarray, 
                          extrema: np.ndarray, 
                          gap_size_deg: float) -> List[Tuple[int, int]]:
    """
    查找向上反转的辅助函数。
    
    Args:
        steer_angle_deg: 方向盘角度序列
        extrema: 极值点索引数组
        gap_size_deg: 间隙大小阈值
        
    Returns:
        List[Tuple[int, int]]: 反转点对的列表，每个元组包含起始和结束索引
    """
    
    if len(extrema) == 0:
        return []
    
    upward_reversals = []
    i = 0
    
    for j in range(len(extrema)):
        if (steer_angle_deg[extrema[j]] - steer_angle_deg[extrema[i]]) > gap_size_deg:
            upward_reversals.append((extrema[i], extrema[j]))
            i = j
        elif steer_angle_deg[extrema[j]] <= steer_angle_deg[extrema[i]]:
            i = j
    
    return upward_reversals





# 示例使用
if __name__ == "__main__":
    # 创建测试数据
    t_list = np.linspace(0, 10, 1000)
    # 模拟方向盘角度：正弦波加噪声
    sw_data = 20 * np.sin(0.5 * t_list) + 10 * np.sin(2 * t_list) + 2 * np.random.randn(len(t_list))
    
    # 计算SWRR
    swrr = steering_wheel_reversal_rate(
        sw_sequence_deg=sw_data,
        timestamp=t_list,
        gap_size_deg=5.0,
        lowpass_cutoff_freq_hz=2.0,
        debug=True
    )
    
    print(f"\n最终结果: SWRR = {swrr:.2f} 反转/分钟")