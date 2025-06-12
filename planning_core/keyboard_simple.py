#!/usr/bin/env python3

import rospy
import argparse
import pygame as pg

from pygame.locals import *
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped


class SimpleKeyboardHandler:
    """简化的键盘处理器，只处理w,a,s,d,b按键功能"""
    
    def __init__(self):
        """初始化键盘处理器
        
        初始化ROS发布器和命令计数器
        """
        self.cmd_count = 0
        self.joy_pub = rospy.Publisher("/joy", Joy, queue_size=10)
        self.ego_cmd_pub = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=10
        )

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
        """发布ego车辆启动命令"""
        cmd = PoseStamped()
        cmd.header.frame_id = "map"
        cmd.header.stamp = rospy.Time.now()
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