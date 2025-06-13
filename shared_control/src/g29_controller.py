#!/usr/bin/env python3

import rospy
import pygame as pg
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
from carla_msgs.msg import CarlaEgoVehicleControl


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
        self.alpha = rospy.get_param('~alpha', 0.0)  # 默认各占50%
        
        # ROS发布器
        self.joy_pub = rospy.Publisher("/joy", Joy, queue_size=10)
        self.ego_cmd_pub = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=10
        )
        
        # 车辆控制命令发布器 (最终输出)
        control_topic_out = f"/carla/{self.ego_vehicle_name}/vehicle_control_cmd"
        self.vehicle_control_pub = rospy.Publisher(
            control_topic_out, CarlaEgoVehicleControl, queue_size=10
        )
        
        # 订阅机器控制信号
        control_topic_in = f"/carla/{self.ego_vehicle_name}/vehicle_control_cmd_tmp"
        self.machine_control_sub = rospy.Subscriber(
            control_topic_in, CarlaEgoVehicleControl, self.machine_control_callback
        )
        
        # 存储最新的机器控制命令
        self.latest_machine_control = CarlaEgoVehicleControl()
        self.machine_control_received = False
        
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
        
        # G29按键映射 (根据实际测试可能需要调整)
        self.button_mapping = {
            3: 3,   # w键对应的功能 -> G29按键3
            1: 1,   # a键对应的功能 -> G29按键1  
            0: 0,   # s键对应的功能 -> G29按键0
            2: 2,   # d键对应的功能 -> G29按键2
            23: 23  # b键对应的功能 -> G29按键23
        }
        
        # 上一次按键状态，用于检测按键按下事件
        self.prev_button_states = [False] * self.joystick.get_numbuttons()
        
        # 踏板和方向盘的上一次值
        self.prev_steering = 0.0
        self.prev_throttle = 0.0
        self.prev_brake = 0.0
        
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
    
    def read_steering_wheel(self):
        """读取方向盘转向值
        
        Returns:
            float: 方向盘转向值 (-1.0 到 1.0)
        """
        if self.joystick.get_numaxes() > 0:
            return self.joystick.get_axis(0)  # 通常轴0是方向盘
        return 0.0
    
    def read_throttle_pedal(self):
        """读取油门踏板值
        
        Returns:
            float: 油门踏板值 (0.0 到 1.0)
        """
        if self.joystick.get_numaxes() > 1:
            # G29油门踏板通常在轴1或轴2，需要根据实际情况调整
            raw_value = self.joystick.get_axis(1)
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
            raw_value = self.joystick.get_axis(2)
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
    
    def get_human_control_input(self):
        """获取人类驾驶员的控制输入
        
        Returns:
            CarlaEgoVehicleControl: 人类驾驶员的控制命令
        """
        human_control = CarlaEgoVehicleControl()
        human_control.header.stamp = rospy.Time.now()
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
    
    def blend_control_signals(self, human_control, machine_control, alpha):
        """混合人类和机器的控制信号
        
        Args:
            human_control (CarlaEgoVehicleControl): 人类控制输入
            machine_control (CarlaEgoVehicleControl): 机器控制输入
            alpha (float): 人类控制权重 (0.0-1.0)
            
        Returns:
            CarlaEgoVehicleControl: 混合后的控制命令
        """
        blended_control = CarlaEgoVehicleControl()
        blended_control.header.stamp = rospy.Time.now()
        blended_control.header.frame_id = "base_link"
        
        # 线性加权混合
        blended_control.steer = alpha * human_control.steer + (1.0 - alpha) * machine_control.steer
        # blended_control.throttle = alpha * human_control.throttle + (1.0 - alpha) * machine_control.throttle
        # blended_control.brake = alpha * human_control.brake + (1.0 - alpha) * machine_control.brake
        blended_control.throttle = machine_control.throttle
        blended_control.brake = machine_control.brake
        
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
    
    def process_analog_inputs(self):
        """处理模拟输入（方向盘、踏板）并发布共享控制命令
        
        读取方向盘和踏板的值，与机器控制信号混合后发布最终控制命令
        """
        steering = self.read_steering_wheel()
        throttle = self.read_throttle_pedal()
        brake = self.read_brake_pedal()
        
        # 检测显著变化时打印信息（可选）
        if abs(steering - self.prev_steering) > 0.01:
            rospy.logdebug(f"Human Steering: {steering:.2f}")
            self.prev_steering = steering
        
        if abs(throttle - self.prev_throttle) > 0.01:
            rospy.logdebug(f"Human Throttle: {throttle:.2f}")
            self.prev_throttle = throttle
        
        if abs(brake - self.prev_brake) > 0.01:
            rospy.logdebug(f"Human Brake: {brake:.2f}")
            self.prev_brake = brake
        
        # 获取人类控制输入
        human_control = self.get_human_control_input()
        
        # 如果收到了机器控制信号，进行混合
        if self.machine_control_received:
            final_control = self.blend_control_signals(
                human_control, self.latest_machine_control, self.alpha
            )
            rospy.logdebug(f"Blended control - Steer: {final_control.steer:.2f}, "
                          f"Throttle: {final_control.throttle:.2f}, Brake: {final_control.brake:.2f}")
        else:
            # 如果没有机器控制信号，只使用人类控制
            final_control = human_control
            rospy.logdebug("Using human control only (no machine signal)")
        
        # 发布最终控制命令
        self.vehicle_control_pub.publish(final_control)
    
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
    rate = rospy.Rate(30)  # 30Hz更新频率，适合实时控制
    
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