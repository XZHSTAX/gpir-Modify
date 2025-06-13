#!/usr/bin/env python3

import rospy
import pygame as pg
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped


class G29Controller:
    """Logitech G29方向盘控制器
    
    读取G29方向盘的按键、踏板和方向盘输入，并发布相应的ROS消息
    """
    
    def __init__(self):
        """初始化G29控制器
        
        初始化ROS发布器、pygame joystick和命令计数器
        """
        self.cmd_count = 0
        
        # ROS发布器
        self.joy_pub = rospy.Publisher("/joy", Joy, queue_size=10)
        self.ego_cmd_pub = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=10
        )
        
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
    
    def process_analog_inputs(self):
        """处理模拟输入（方向盘、踏板）
        
        读取方向盘和踏板的值，为将来的功能扩展做准备
        """
        steering = self.read_steering_wheel()
        throttle = self.read_throttle_pedal()
        brake = self.read_brake_pedal()
        
        # 检测显著变化时打印信息（可选）
        if abs(steering - self.prev_steering) > 0.01:
            rospy.logdebug(f"Steering: {steering:.2f}")
            self.prev_steering = steering
        
        if abs(throttle - self.prev_throttle) > 0.01:
            rospy.logdebug(f"Throttle: {throttle:.2f}")
            self.prev_throttle = throttle
        
        if abs(brake - self.prev_brake) > 0.01:
            rospy.logdebug(f"Brake: {brake:.2f}")
            self.prev_brake = brake
        
        # TODO: 在这里添加方向盘和踏板的具体处理逻辑
        # 例如：发布车辆控制命令、调整参考速度等
    
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