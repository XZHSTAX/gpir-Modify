#!/usr/bin/env python3

import rospy
import argparse
import pygame as pg
import tf.transformations as tf_trans

from pygame.locals import *
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped, Quaternion


class SimpleKeyboardHandler:
    """简化的键盘处理器，只处理w,a,s,d,b按键功能"""
    
    def __init__(self):
        """初始化键盘处理器
        
        初始化ROS发布器和命令计数器，获取launch文件中的位置参数
        """
        self.cmd_count = 0
        self.joy_pub = rospy.Publisher("/joy", Joy, queue_size=10)
        self.ego_cmd_pub = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=10
        )
        
        # 从launch文件获取目标位置参数 (格式: "x,y,z,roll,pitch,yaw")
        goal_point_str = rospy.get_param('~goal_point', None)
        self.target_x = None
        self.target_y = None
        self.target_z = None
        self.target_roll = None
        self.target_pitch = None
        self.target_yaw = None
        
        if goal_point_str is not None:
            try:
                # 解析逗号分隔的参数字符串
                params = [float(x.strip()) for x in goal_point_str.split(',')]
                if len(params) == 6:
                    self.target_x = params[0]
                    self.target_y = params[1]
                    self.target_z = params[2]
                    self.target_roll = params[3]
                    self.target_pitch = params[4]
                    self.target_yaw = params[5]
                    rospy.loginfo(f"Parsed goal_point: x={self.target_x}, y={self.target_y}, z={self.target_z}, "
                                 f"roll={self.target_roll}, pitch={self.target_pitch}, yaw={self.target_yaw}")
                else:
                    rospy.logwarn(f"Invalid goal_point format. Expected 6 values (x,y,z,roll,pitch,yaw), got {len(params)}")
            except ValueError as e:
                rospy.logerr(f"Failed to parse goal_point parameter: {e}")

    def init_joy(self):
        """初始化Joy消息
        
        Returns:
            Joy: 初始化的Joy消息对象
        """
        joy = Joy()
        joy.header.frame_id = "map"
        joy.header.stamp = rospy.Time.now()
        for i in range(8):
            joy.axes.append(0)
        for i in range(11):
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
        """发布ego车辆启动命令
        
        如果launch文件中设置了目标位置参数，则使用这些参数；
        否则使用默认的空位置信息
        """
        cmd = PoseStamped()
        cmd.header.frame_id = "map"
        cmd.header.stamp = rospy.Time.now()
        
        # 检查是否有从launch文件传入的参数
        if all(param is not None for param in [self.target_x, self.target_y, self.target_z]):
            # 设置位置信息
            cmd.pose.position.x = self.target_x
            cmd.pose.position.y = self.target_y
            cmd.pose.position.z = self.target_z
            
            # 如果有姿态参数，则设置姿态信息
            if all(param is not None for param in [self.target_roll, self.target_pitch, self.target_yaw]):
                # 将欧拉角转换为四元数
                quaternion = tf_trans.quaternion_from_euler(
                    self.target_roll, self.target_pitch, self.target_yaw
                )
                cmd.pose.orientation = Quaternion(
                    x=quaternion[0],
                    y=quaternion[1], 
                    z=quaternion[2],
                    w=quaternion[3]
                )
            else:
                # 如果没有姿态参数，使用默认姿态（朝向正前方）
                cmd.pose.orientation = Quaternion(x=0, y=0, z=0, w=1)
                
            rospy.loginfo(f"Publishing ego start command with target position: "
                         f"x={self.target_x}, y={self.target_y}, z={self.target_z}, "
                         f"roll={self.target_roll}, pitch={self.target_pitch}, yaw={self.target_yaw}")
        else:
            rospy.loginfo("Publishing ego start command with default empty position")
            
        self.ego_cmd_pub.publish(cmd)

    def update(self):
        """更新键盘输入处理
        
        处理pygame事件，检测w,a,s,d,b按键并发布相应的Joy消息
        """
        for event in pg.event.get():
            if event.type == pg.KEYDOWN:
                joy = self.init_joy()
                if event.key == pg.K_w:
                    print("{}: Increase reference speed.".format(self.cmd_count))
                    joy.buttons[3] = 1
                    self.publish_joy(joy)
                elif event.key == pg.K_s:
                    print("{}: Decrease reference speed.".format(self.cmd_count))
                    joy.buttons[0] = 1
                    self.publish_joy(joy)
                elif event.key == pg.K_a:
                    print("{}: Suggest left lane change.".format(self.cmd_count))
                    joy.buttons[2] = 1
                    self.publish_joy(joy)
                elif event.key == pg.K_d:
                    print("{}: Suggest right lane change.".format(self.cmd_count))
                    joy.buttons[1] = 1
                    self.publish_joy(joy)
                elif event.key == pg.K_b:
                    print("{}: Trigger ego start command.".format(self.cmd_count))
                    self.publish_ego_start_cmd()


def main():
    """主函数
    
    初始化ROS节点、pygame和键盘处理器，运行主循环
    """
    argparser = argparse.ArgumentParser(description="Simple keyboard to joy converter")
    argparser.add_argument("--img", type=str, required=True, help="Image file path")
    args, unknown = argparser.parse_known_args()

    rospy.init_node("simple_key2joy")
    rate = rospy.Rate(10)

    pg.init()
    screen = pg.display.set_mode((886, 542))
    img = pg.image.load(args.img)
    keyboard_handler = SimpleKeyboardHandler()

    try:
        while not rospy.is_shutdown():
            keyboard_handler.update()
            screen.blit(img, (1, 1))
            pg.display.flip()
            rate.sleep()
    finally:
        print("Shutting down simple keyboard handler")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        pg.quit()