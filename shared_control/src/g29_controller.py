#!/usr/bin/env python3

from platform import machine
import rospy
import pygame as pg
import numpy as np
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Float64
from carla_msgs.msg import CarlaEgoVehicleControl
from ackermann_msgs.msg import AckermannDrive
from ros_g29_force_feedback.msg import ForceFeedback
from derived_object_msgs.msg import ObjectArray
from tf.transformations import euler_from_quaternion
import pandas as pd
from TwoDimTTC import TTC
from authority_allocator import AuthorityAllocator
from safety_filter import cbf_filter
from pid_controller import PIDController
from filters import LowPassFilterStream

class G29Controller:
    """Logitech G29方向盘控制器
    
    读取G29方向盘的按键、踏板和方向盘输入，并发布相应的ROS消息
    """
    
    def __init__(self):
        """初始化G29控制器
        
        初始化ROS发布器、订阅器、pygame joystick和命令计数器
        """
        self.cmd_count = 0
        
        # 获取ego_vehicle名称参数
        self.ego_vehicle_name = rospy.get_param('~ego_vehicle_name', 'ego_vehicle')
        
        # 共享控制权重参数 (alpha: 人类控制权重, 1-alpha: 机器控制权重)
        self.alpha = rospy.get_param('~alpha', 0.0)  # 默认机器有完全权限
        
        # 权限移交延迟时间参数 (毫秒)
        self.transition_delay_ms = rospy.get_param('~transition_delay_ms', 5000)  # 默认5秒
        
        # 权限移交延迟时间参数 (秒) - 从launch文件传入
        self.transition_delay = rospy.get_param('~transition_delay', 5.0)  # 默认5秒
        
        # 目标接收状态跟踪
        self.goal_received = False
        self.goal_timer = None
        
        # 初始化权限分配器
        strategy_name = rospy.get_param('~authority_strategy', 'ConstantAlpha')

        self.next_strategy_name = rospy.get_param('~next_strategy_name', 'FlexibleTransition')
        
        # 创建权限分配器并直接传入策略名称
        self.authority_allocator = AuthorityAllocator(initial_strategy_name=strategy_name)
        
        # ROS发布器
        self.joy_pub = rospy.Publisher("/joy", Joy, queue_size=10)
        self.ego_cmd_pub = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=10
        )
        
        # 车辆控制命令发布器 (最终输出)
        control_topic_out = f"/carla/{self.ego_vehicle_name}/vehicle_control_cmd"
        machine_control_topic_out = f"/carla/{self.ego_vehicle_name}/vehicle_control_cmd_machine"
        self.vehicle_control_pub = rospy.Publisher(
            control_topic_out, CarlaEgoVehicleControl, queue_size=10
        )
        
        # 人类控制输入发布器
        self.human_control_pub = rospy.Publisher(
            "/human_control_input", CarlaEgoVehicleControl, queue_size=10
        )

        # 混合控制输入发布器
        self.mix_control_pub = rospy.Publisher(
            "/mixControl", CarlaEgoVehicleControl, queue_size=10
        )

        self.machine_control_pub = rospy.Publisher(
            machine_control_topic_out, CarlaEgoVehicleControl, queue_size=10
        )
        
        # 力反馈发布器
        self.force_feedback_pub = rospy.Publisher(
            "/ff_target", ForceFeedback, queue_size=10
        )
        
        # 策略命令发布器
        self.strategy_command_pub = rospy.Publisher(
            "/shared_control/strategy_command", String, queue_size=10
        )
        
        # 订阅机器控制信号
        control_topic_in = f"/carla/{self.ego_vehicle_name}/vehicle_control_cmd_tmp"
        self.machine_control_sub = rospy.Subscriber(
            control_topic_in, CarlaEgoVehicleControl, self.machine_control_callback
        )
        
        # 订阅目标位置信号
        self.goal_sub = rospy.Subscriber(
            "/move_base_simple/goal", PoseStamped, self.goal_callback
        )
        
        # 订阅外部扭矩和机器扭矩
        self.external_torque_sub = rospy.Subscriber(
            "external_torque", Float64, self.external_torque_callback
        )
        self.machine_torque_sub = rospy.Subscriber(
            "machine_torque", Float64, self.machine_torque_callback
        )

        # 订阅ackermann指令
        ackermann_cmd_topic = f"/carla/{self.ego_vehicle_name}/ackermann_cmd"
        self.ackermann_cmd_sub = rospy.Subscriber(
            ackermann_cmd_topic, AckermannDrive, self.ackermann_cmd_callback
        )

        # 订阅场景中所有车辆的信息
        self.objects_sub = rospy.Subscriber(
            "/carla/objects", ObjectArray, self.objects_callback
        )

        # 存储最新的机器控制命令
        self.latest_machine_control = CarlaEgoVehicleControl()
        self.machine_control_received = False
        self.external_torque = 0.0
        self.machine_torque = 0.0

        # 初始化低通滤波器
        # 假设采样率为100Hz，截止频率为5Hz
        sample_rate = 50  # Hz
        cutoff_freq = 3  # Hz
        self.external_torque_filter = LowPassFilterStream(cutoff_freq,sample_rate)
        self.machine_torque_filter = LowPassFilterStream(cutoff_freq,sample_rate)
        self.latest_ackermann_cmd = AckermannDrive()
        self.ego_vehicle_state = np.zeros(3)  # [x, y, psi]
        self.other_vehicles_state = np.empty((0, 3))  # [x, y, psi]
        self.ego_vehicle_speed = 0.0
        self.ego_vehicle_acc_lat_lon = [0,0]
        self.speed_controller = PIDController(1.5, 0, 0.05)
        self.speed_controller_alpha = 0.2

        self.final_control = CarlaEgoVehicleControl()

        # For TTC calculation
        self.min_ttc = np.inf
        self.ego_vehicle_full_state = {}
        self.other_vehicles_full_state = []
        
        # 初始化pygame和joystick
        pg.init()
        pg.joystick.init()
        
        # 检查是否有joystick连接
        if pg.joystick.get_count() == 0:
            rospy.logerr("No joystick detected! Please connect Logitech G29.")
            return
        
        # 初始化第一个joystick (G29)
        self.joystick = pg.joystick.Joystick(0)
        self.joystick.init()
        
        rospy.loginfo(f"Initialized joystick: {self.joystick.get_name()}")
        rospy.loginfo(f"Number of axes: {self.joystick.get_numaxes()}")
        rospy.loginfo(f"Number of buttons: {self.joystick.get_numbuttons()}")
        
        rospy.loginfo(f"G29 Shared Controller initialized for vehicle: {self.ego_vehicle_name}")
        rospy.loginfo(f"Control weight alpha (human): {self.alpha:.2f}")
        rospy.loginfo(f"Subscribing to: {control_topic_in}")
        rospy.loginfo(f"Publishing to: {control_topic_out}")
    
    def init_joy(self):
        """初始化Joy消息
        
        Returns:
            Joy: 初始化的Joy消息对象
        """
        joy = Joy()
        joy.header.frame_id = "map"
        joy.header.stamp = rospy.Time.now()
        
        # 初始化axes和buttons数组
        for i in range(8):
            joy.axes.append(0)
        for i in range(32):  # G29有更多按键，扩展到32个
            joy.buttons.append(0)
        
        return joy
    
    def publish_joy(self, joy):
        """发布Joy消息
        
        Args:
            joy (Joy): 要发布的Joy消息
        """
        self.cmd_count += 1
        self.joy_pub.publish(joy)
    
    def publish_ego_start_cmd(self):
        """发布ego车辆启动命令"""
        cmd = PoseStamped()
        cmd.header.frame_id = "map"
        cmd.header.stamp = rospy.Time.now()
        self.ego_cmd_pub.publish(cmd)
    
    def handle_button_press(self, button_id):
        """处理按键按下事件
        
        Args:
            button_id (int): 按下的按键ID
        """
        joy = self.init_joy()
        
        if button_id == 3:  # 对应w键功能
            print(f"{self.cmd_count}: Increase reference speed.")
            joy.buttons[3] = 1
            self.publish_joy(joy)
        elif button_id == 0:  # 对应s键功能
            print(f"{self.cmd_count}: Decrease reference speed.")
            joy.buttons[0] = 1
            self.publish_joy(joy)
        elif button_id == 1:  # 对应a键功能
            print(f"{self.cmd_count}: Suggest left lane change.")
            joy.buttons[2] = 1
            self.publish_joy(joy)
        elif button_id == 2:  # 对应d键功能
            print(f"{self.cmd_count}: Suggest right lane change.")
            joy.buttons[1] = 1
            self.publish_joy(joy)
        elif button_id == 23:  # 对应b键功能
            print(f"{self.cmd_count}: Trigger ego start command.")
            self.publish_ego_start_cmd()
    
    def objects_callback(self, msg):
        """
        处理来自/carla/objects话题的数据，更新车辆状态

        Args:
            msg (derived_object_msgs.msg.ObjectArray): 包含场景中所有对象信息的消息
        """
        if not msg.objects:
            return

        try:
            # 找到ID最小的车辆作为自车
            ego_vehicle_obj = min(msg.objects, key=lambda obj: obj.id)
        except ValueError:
            return

        # 提取自车状态
        ego_x = ego_vehicle_obj.pose.position.x
        ego_y = ego_vehicle_obj.pose.position.y
        orientation = ego_vehicle_obj.pose.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, ego_psi = euler_from_quaternion(quaternion)
        self.ego_vehicle_state = np.array([ego_x, ego_y, ego_psi])
        vx = ego_vehicle_obj.twist.linear.x
        vy = ego_vehicle_obj.twist.linear.y

        self.ego_vehicle_speed = np.sqrt(vx**2 + vy**2)

        ax = ego_vehicle_obj.accel.linear.x
        ay = ego_vehicle_obj.accel.linear.y
        cos_yaw = np.cos(ego_psi)
        sin_yaw = np.sin(ego_psi)
        self.ego_vehicle_acc_lat_lon[0] = ax * cos_yaw + ay * sin_yaw
        self.ego_vehicle_acc_lat_lon[1] = ay * cos_yaw -ax * sin_yaw

        # For TTC calculation
        self.ego_vehicle_full_state = {
            'x': ego_x,
            'y': ego_y,
            'vx': vx,
            'vy': vy,
            'hx': cos_yaw,
            'hy': sin_yaw,
            'length': ego_vehicle_obj.shape.dimensions[0] if len(ego_vehicle_obj.shape.dimensions) > 0 else 4.5, # default car length
            'width': ego_vehicle_obj.shape.dimensions[1] if len(ego_vehicle_obj.shape.dimensions) > 1 else 1.8, # default car width
        }

        # 提取其他车辆状态
        other_vehicles = []
        self.other_vehicles_full_state = []
        for obj in msg.objects:
            if obj.id != ego_vehicle_obj.id:
                x = obj.pose.position.x
                y = obj.pose.position.y
                orientation = obj.pose.orientation
                quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
                _, _, psi = euler_from_quaternion(quaternion)
                other_vehicles.append([x, y, psi])
                
                # For TTC calculation
                self.other_vehicles_full_state.append({
                    'x': x,
                    'y': y,
                    'vx': obj.twist.linear.x,
                    'vy': obj.twist.linear.y,
                    'hx': np.cos(psi),
                    'hy': np.sin(psi),
                    'length': obj.shape.dimensions[0] if len(obj.shape.dimensions) > 0 else 4.5,
                    'width': obj.shape.dimensions[1] if len(obj.shape.dimensions) > 1 else 1.8,
                })
        
        if other_vehicles:
            self.other_vehicles_state = np.array(other_vehicles)
        else:
            self.other_vehicles_state = np.empty((0, 3))

        self.calculate_and_update_ttc()

    def calculate_and_update_ttc(self):
        """
        计算并更新自车与场景中其他车辆的最小TTC
        """
        if not self.ego_vehicle_full_state or not self.other_vehicles_full_state:
            self.min_ttc = np.inf
            return

        ttc_values = []
        for other_vehicle_state in self.other_vehicles_full_state:
            data = {
                'x_i': self.ego_vehicle_full_state['x'],
                'y_i': self.ego_vehicle_full_state['y'],
                'vx_i': self.ego_vehicle_full_state['vx'],
                'vy_i': self.ego_vehicle_full_state['vy'],
                'hx_i': self.ego_vehicle_full_state['hx'],
                'hy_i': self.ego_vehicle_full_state['hy'],
                'length_i': self.ego_vehicle_full_state['length'],
                'width_i': self.ego_vehicle_full_state['width'],
                'x_j': other_vehicle_state['x'],
                'y_j': other_vehicle_state['y'],
                'vx_j': other_vehicle_state['vx'],
                'vy_j': other_vehicle_state['vy'],
                'hx_j': other_vehicle_state['hx'],
                'hy_j': other_vehicle_state['hy'],
                'length_j': other_vehicle_state['length'],
                'width_j': other_vehicle_state['width'],
            }
            df = pd.DataFrame([data])
            # The TTC function returns a numpy array
            ttc = TTC(df, toreturn='values')
            if ttc.size > 0:
                ttc_values.append(ttc[0])

        if ttc_values:
            self.min_ttc = min(ttc_values)
        else:
            self.min_ttc = np.inf

    def read_steering_wheel(self):
        """读取方向盘转向值
        
        Returns:
            float: 方向盘转向值 (-1.0 到 1.0)
        """
        if self.joystick.get_numaxes() > 0:
            return self.joystick.get_axis(0)  # 通常轴0是方向盘
        return 0.0

    def ackermann_cmd_callback(self, msg):
        """Ackermann指令回调函数

        Args:
            msg (AckermannDrive): 接收到的Ackermann指令消息
        """
        self.latest_ackermann_cmd = msg
    
    def read_throttle_pedal(self):
        """读取油门踏板值
        
        Returns:
            float: 油门踏板值 (0.0 到 1.0)
        """
        if self.joystick.get_numaxes() > 1:
            # G29油门踏板通常在轴1或轴2，需要根据实际情况调整
            raw_value = self.joystick.get_axis(2)
            # 将-1到1的范围转换为0到1
            return (1.0 - raw_value) / 2.0
        return 0.0
    
    def read_brake_pedal(self):
        """读取刹车踏板值
        
        Returns:
            float: 刹车踏板值 (0.0 到 1.0)
        """
        if self.joystick.get_numaxes() > 2:
            # G29刹车踏板通常在轴2或轴3，需要根据实际情况调整
            raw_value = self.joystick.get_axis(3)
            # 将-1到1的范围转换为0到1
            return (1.0 - raw_value) / 2.0
        return 0.0
    
    def machine_control_callback(self, msg):
        """机器控制信号回调函数
        
        Args:
            msg (CarlaEgoVehicleControl): 机器生成的控制命令
        """
        self.latest_machine_control = msg
        self.machine_control_received = True
    
    def external_torque_callback(self, msg):
        """外部扭矩回调函数
        
        Args:
            msg (Float64): 外部扭矩消息
        """
        self.external_torque = self.external_torque_filter.filter(msg.data)

    def machine_torque_callback(self, msg):
        """机器扭矩回调函数
        
        Args:
            msg (Float64): 机器扭矩消息
        """
        self.machine_torque = self.machine_torque_filter.filter(msg.data)

    def goal_callback(self, msg):
        """目标位置回调函数
        
        在第一次接收到/move_base_simple/goal topic时，启动定时器
        在指定延迟后发布FlexibleTransition策略命令
        
        Args:
            msg (PoseStamped): 目标位置消息
        """
        if not self.goal_received:
            self.goal_received = True
            rospy.loginfo(f"First goal received, will publish {self.next_strategy_name} command after {self.transition_delay} seconds")
            
            # 如果已有定时器在运行，先取消
            if self.goal_timer is not None:
                self.goal_timer.shutdown()
            
            # 创建新的定时器
            self.goal_timer = rospy.Timer(
                rospy.Duration(self.transition_delay), 
                self.goal_timer_callback, 
                oneshot=True
            )
    
    def goal_timer_callback(self, event):
        """目标定时器回调函数
        
        在延迟时间后发布下一个策略的命令
        
        Args:
            event: 定时器事件
        """
        strategy_msg = String()
        strategy_msg.data = self.next_strategy_name
        self.strategy_command_pub.publish(strategy_msg)
        rospy.loginfo(f"Published {self.next_strategy_name} strategy command")
        
        # 清理定时器
        if self.goal_timer is not None:
            self.goal_timer.shutdown()
            self.goal_timer = None
    
    def get_human_control_input(self,time_now):
        """获取人类驾驶员的控制输入
        
        Returns:
            CarlaEgoVehicleControl: 人类驾驶员的控制命令
        """
        human_control = CarlaEgoVehicleControl()
        human_control.header.stamp = time_now
        human_control.header.frame_id = "base_link"
        
        # 读取G29输入
        steering = self.read_steering_wheel()
        throttle = self.read_throttle_pedal()
        brake = self.read_brake_pedal()
        
        # 转换为CARLA控制格式
        human_control.steer = steering
        human_control.throttle = throttle
        human_control.brake = brake
        human_control.hand_brake = False
        human_control.reverse = False
        human_control.gear = 1
        human_control.manual_gear_shift = False
        
        return human_control
    
    def build_authority_context(self, human_control,start_transition=False):
        """构建权限分配所需的上下文信息
        
        Args:
            human_control (CarlaEgoVehicleControl): 人类控制输入
            
        Returns:
            Dict[str, Any]: 包含各种上下文信息的字典
        """
        context = {
            'human_control': human_control,
            'machine_control': self.latest_machine_control if self.machine_control_received else None,
            'steering_angle': human_control.steer,
            'throttle_input': human_control.throttle,
            'brake_input': human_control.brake,
            'time_delta': (rospy.Time.now() - self.authority_allocator.current_strategy.last_update_time).to_sec(),
            'machine_control_available': self.machine_control_received,
            'start_transition': start_transition,
            'T':self.external_torque,
            'Tt':self.machine_torque,
            'v_exp': 0,
            'ay':self.ego_vehicle_acc_lat_lon[1],
            't_ttc': self.min_ttc
        }
        return context
    
    def blend_control_signals(self, human_control, machine_control, alpha,time_now):
        """混合人类和机器的控制信号
        
        Args:
            human_control (CarlaEgoVehicleControl): 人类控制输入
            machine_control (CarlaEgoVehicleControl): 机器控制输入
            alpha (float): 人类控制权重 (0.0-1.0)
            
        Returns:
            CarlaEgoVehicleControl: 混合后的控制命令
        """
        blended_control = CarlaEgoVehicleControl()
        blended_control.header.stamp = time_now
        blended_control.header.frame_id = "base_link"
        
        # 线性加权混合
        blended_control.steer = alpha * human_control.steer + (1.0 - alpha) * machine_control.steer
        # blended_control.throttle = alpha * human_control.throttle + (1.0 - alpha) * machine_control.throttle
        # blended_control.brake = alpha * human_control.brake + (1.0 - alpha) * machine_control.brake
        if self.authority_allocator.current_strategy.name == 'HumanMachineCollaboration':
            blended_control.brake = machine_control.brake
            blended_control.throttle = machine_control.throttle
        else:
            if human_control.brake > machine_control.brake + 0.01:
                blended_control.brake = human_control.brake
            else:
                blended_control.brake = machine_control.brake

            if human_control.throttle > machine_control.throttle + 0.01:
                blended_control.throttle = human_control.throttle
                blended_control.brake = 0
            elif human_control.brake > machine_control.brake + 0.01:
                blended_control.throttle = 0
            else:
                blended_control.throttle = machine_control.throttle
        
        # 布尔值采用人类优先策略
        blended_control.hand_brake = human_control.hand_brake or machine_control.hand_brake
        blended_control.reverse = human_control.reverse or machine_control.reverse
        
        # 档位和手动换挡采用机器控制
        blended_control.gear = machine_control.gear if machine_control.gear != 0 else human_control.gear
        blended_control.manual_gear_shift = machine_control.manual_gear_shift
        
        # 确保控制值在有效范围内
        blended_control.steer = max(-1.0, min(1.0, blended_control.steer))
        blended_control.throttle = max(0.0, min(1.0, blended_control.throttle))
        blended_control.brake = max(0.0, min(1.0, blended_control.brake))
        
        return blended_control

    def _apply_safety_filter(self, human_control, v_exp, mix_control,dt = 0.02,k_delta=np.pi/6):
        """应用安全过滤器。

        根据人类输入和车辆状态，对控制指令应用安全过滤器。

        Args:
            human_control: 来自人类驾驶员的控制输入。
            v_exp (float): 期望速度。
            final_control: 将要被修改的最终控制指令。
            dt: 该函数被调用的时间间隔，实际上就是该节点的运行频率的倒数
            k_delta: 方向盘从[-1,1]映射到前轮转角的比例系数

        Returns:
            tuple: 包含更新后的期望速度和最终控制指令的元组。
        """
        # 1. 根据踏板深度重新设置期望速度
        Delta_acc = human_control.throttle - self.latest_machine_control.throttle
        Delta_brake = human_control.brake - self.latest_machine_control.brake

        # 如果人类的对油门的控制大于机器的控制，则需要根据差值修改期望速度
        # 这里没有写如果同时踩下刹车和油门的处理逻辑，因为实验中没人这么做
        if Delta_acc > 0.01:
            v_exp = v_exp + Delta_acc * 10
        elif Delta_brake > 0.01:
            v_exp = v_exp - Delta_brake * 10
            v_exp = max(v_exp, 0)

        # 2. 安全过滤器
        # 准备 cbf_filter 的输入
        # 假设轴距为 2.875 米
        wheelbase = 2.875
        omega_mix = v_exp * np.tan(mix_control.steer*k_delta) / wheelbase # 注意final_control.steer的输入值是[-1,1]，需要转换为前轮转角区间[-30,30]，即[-pi/6,pi/6]
        u_star = np.array([v_exp, omega_mix])
        p_ego = self.ego_vehicle_state
        p_other = self.other_vehicles_state

        # 调用安全过滤器
        if p_other.size > 0:
            u_safe = cbf_filter(u_star, p_ego, p_other)
            v_safe, omega_safe = u_safe[0], u_safe[1]

            # 更新 final_control
            v_safe_control = self.speed_controller.control(v_safe - self.ego_vehicle_speed, dt)
            if v_safe_control >= 0:
                mix_control.throttle = self.speed_controller_alpha * min(v_safe_control / 4.0, 1.0) + (1 - self.speed_controller_alpha) * self.final_control.throttle
                mix_control.brake = 0
            else:
                if(abs(v_safe_control)/15 > 0.1):
                    mix_control.throttle = 0
                    mix_control.brake = (1 - self.speed_controller_alpha) * min(abs(v_safe_control) / 15, 1.0) + self.speed_controller_alpha * self.final_control.brake
                else:
                    mix_control.throttle = 0
                    mix_control.brake = 0

            # 根据安全角速度更新转向
            # steer = arctan(omega * L / v)
            # 当v很小时，这个计算不稳定，需要处理
            if v_safe > 0.1:
                safe_steer = np.arctan(omega_safe * wheelbase / v_safe) /k_delta # 注意需要将前轮转角区间[-30,30]转换为[-1,1]
                mix_control.steer = np.clip(safe_steer, -1.0, 1.0)
            else:
                # 速度很低时，保持原转向或置零
                mix_control.steer = 0.0

            # rospy.loginfo(f"Original u*: {u_star}, Safe u: {u_safe}")
        else:
            rospy.loginfo("No other vehicles, skipping safety filter.")
        
        return v_exp, mix_control
    
    def process_analog_inputs(self):
        """处理模拟输入（方向盘、踏板）并发布共享控制命令
        
        读取方向盘和踏板的值，与机器控制信号混合后发布最终控制命令
        """
        time_now = rospy.Time.now()
        # 获取人类控制输入
        human_control = self.get_human_control_input(time_now)
        # human_control.steer = 0
        
        # 构建权限分配上下文
        context = self.build_authority_context(human_control)
        
        # 动态更新alpha值
        self.alpha = self.authority_allocator.update_alpha(context)
        
        # 如果收到了机器控制信号，并且人类的权限小于0.99
        if self.machine_control_received and self.alpha < 0.99:
            mix_control = self.blend_control_signals(
                human_control, self.latest_machine_control, self.alpha,time_now
            )
            self.mix_control_pub.publish(mix_control)
            rospy.logdebug(f"Blended control - Steer: {mix_control.steer:.2f}, "
                          f"Throttle: {mix_control.throttle:.2f}, Brake: {mix_control.brake:.2f}")
            # 安全过滤器
            v_exp = self.latest_ackermann_cmd.speed
            if self.authority_allocator.current_strategy.name == 'HumanMachineCollaboration':
                v_exp,self.final_control  = self._apply_safety_filter(human_control, v_exp, mix_control)
            else:
                self.final_control = mix_control
            # final_control = mix_control
        else:
            # 如果没有机器控制信号，只使用人类控制
            self.final_control = human_control
            self.mix_control_pub.publish(self.final_control)
            rospy.logdebug("Using human control only (no machine signal)")
        
        # 发布人类控制输入信号
        self.human_control_pub.publish(human_control)
        
        # 发布最终控制命令
        self.vehicle_control_pub.publish(self.final_control)

        self.latest_machine_control.header.stamp = time_now
        self.machine_control_pub.publish(self.latest_machine_control)
        
        # 发布力反馈消息
        self.publish_force_feedback(self.final_control.steer)
    
    def publish_force_feedback(self, steering_position):
        """发布力反馈消息
        
        Args:
            steering_position (float): 方向盘位置 (-1.0 到 1.0)
        """
        ff_msg = ForceFeedback()
        ff_msg.header.stamp = rospy.Time.now()
        ff_msg.header.frame_id = "base_link"
        ff_msg.position = steering_position
        ff_msg.torque = 0.8
        
        self.force_feedback_pub.publish(ff_msg)
    
    def update(self):
        """更新G29输入处理
        
        处理pygame事件，检测按键按下和模拟输入变化
        """
        # 处理pygame事件
        for event in pg.event.get():
            if event.type == pg.JOYBUTTONDOWN:
                self.handle_button_press(event.button)
        # 处理模拟输入
        self.process_analog_inputs()


def main():
    """主函数
    
    初始化ROS节点和G29控制器，运行主循环
    """
    rospy.init_node("g29_controller")
    rate = rospy.Rate(50)  # 30Hz更新频率，适合实时控制
    
    try:
        controller = G29Controller()
        rospy.loginfo("G29 Controller started successfully")
        
        while not rospy.is_shutdown():
            controller.update()
            rate.sleep()
            
    except Exception as e:
        rospy.logerr(f"Error in G29 Controller: {e}")
    finally:
        rospy.loginfo("Shutting down G29 controller")
        pg.quit()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        rospy.loginfo("G29 Controller interrupted by user")
    except rospy.ROSInterruptException:
        rospy.loginfo("G29 Controller interrupted by ROS")
    finally:
        pg.quit()