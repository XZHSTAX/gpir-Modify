#!/usr/bin/env python3

import rospy
import sys
from std_msgs.msg import Float64, String
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from geometry_msgs.msg import Twist


class AuthorityStrategySwitcher:
    """权限分配策略切换器
    
    提供服务接口来动态切换权限分配策略，并监控alpha值变化
    """
    
    def __init__(self):
        """初始化策略切换器"""
        rospy.init_node('authority_strategy_switcher')
        
        # 订阅alpha值变化
        self.alpha_sub = rospy.Subscriber(
            '/shared_control/alpha', Float64, self.alpha_callback
        )
        
        # 发布策略切换命令
        self.strategy_pub = rospy.Publisher(
            '/shared_control/strategy_command', String, queue_size=10
        )
        
        self.current_alpha = 0.5
        self.alpha_history = []
        self.max_history = 100
        
        rospy.loginfo("Authority Strategy Switcher initialized")
        rospy.loginfo("Available commands:")
        rospy.loginfo("  - switch <strategy_name>: 切换到指定策略")
        rospy.loginfo("  - status: 显示当前状态")
        rospy.loginfo("  - monitor: 开始监控alpha值")
        rospy.loginfo("  - help: 显示帮助信息")
        rospy.loginfo("  - quit: 退出程序")
    
    def alpha_callback(self, msg):
        """alpha值变化回调函数
        
        Args:
            msg (Float64): alpha值消息
        """
        self.current_alpha = msg.data
        self.alpha_history.append(self.current_alpha)
        
        if len(self.alpha_history) > self.max_history:
            self.alpha_history.pop(0)
    
    def switch_strategy(self, strategy_name):
        """切换权限分配策略
        
        Args:
            strategy_name (str): 策略名称
        """
        strategy_msg = String()
        strategy_msg.data = strategy_name
        self.strategy_pub.publish(strategy_msg)
        rospy.loginfo(f"Sent strategy switch command: {strategy_name}")
    
    def show_status(self):
        """显示当前状态"""
        rospy.loginfo(f"Current alpha: {self.current_alpha:.3f}")
        if len(self.alpha_history) > 1:
            avg_alpha = sum(self.alpha_history) / len(self.alpha_history)
            rospy.loginfo(f"Average alpha (last {len(self.alpha_history)} values): {avg_alpha:.3f}")
    
    def monitor_alpha(self, duration=10.0):
        """监控alpha值变化
        
        Args:
            duration (float): 监控持续时间（秒）
        """
        rospy.loginfo(f"Monitoring alpha for {duration} seconds...")
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)  # 10Hz
        
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed >= duration:
                break
            
            rospy.loginfo(f"Alpha: {self.current_alpha:.3f} (t={elapsed:.1f}s)")
            rate.sleep()
        
        rospy.loginfo("Monitoring completed")
    
    def show_help(self):
        """显示帮助信息"""
        help_text = """
可用命令:
  switch <strategy>  - 切换权限分配策略
                      可用策略: ConstantAlpha, SteeringBased, EmergencyOverride, Adaptive
  status            - 显示当前alpha值和统计信息
  monitor [time]    - 监控alpha值变化，默认10秒
  help              - 显示此帮助信息
  quit              - 退出程序

示例:
  switch SteeringBased
  monitor 5
  status
        """
        print(help_text)
    
    def run_interactive(self):
        """运行交互式命令行界面"""
        self.show_help()
        
        while not rospy.is_shutdown():
            try:
                command = input("\nEnter command: ").strip().split()
                
                if not command:
                    continue
                
                cmd = command[0].lower()
                
                if cmd == 'quit' or cmd == 'exit':
                    break
                elif cmd == 'help':
                    self.show_help()
                elif cmd == 'status':
                    self.show_status()
                elif cmd == 'switch':
                    if len(command) > 1:
                        self.switch_strategy(command[1])
                    else:
                        rospy.logwarn("Please specify strategy name")
                elif cmd == 'monitor':
                    duration = 10.0
                    if len(command) > 1:
                        try:
                            duration = float(command[1])
                        except ValueError:
                            rospy.logwarn("Invalid duration, using default 10 seconds")
                    self.monitor_alpha(duration)
                else:
                    rospy.logwarn(f"Unknown command: {cmd}. Type 'help' for available commands.")
                    
            except KeyboardInterrupt:
                break
            except EOFError:
                break
        
        rospy.loginfo("Authority Strategy Switcher shutting down")


def main():
    """主函数"""
    try:
        switcher = AuthorityStrategySwitcher()
        
        if len(sys.argv) > 1:
            # 命令行模式
            command = sys.argv[1].lower()
            
            if command == 'switch' and len(sys.argv) > 2:
                switcher.switch_strategy(sys.argv[2])
            elif command == 'status':
                rospy.sleep(1.0)  # 等待订阅建立
                switcher.show_status()
            elif command == 'monitor':
                duration = 10.0
                if len(sys.argv) > 2:
                    try:
                        duration = float(sys.argv[2])
                    except ValueError:
                        pass
                rospy.sleep(1.0)  # 等待订阅建立
                switcher.monitor_alpha(duration)
            else:
                switcher.show_help()
        else:
            # 交互式模式
            switcher.run_interactive()
            
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()