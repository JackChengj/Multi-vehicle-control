#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDrive
import sys, select, termios, tty
import pygame

msg = """
Control robot!
---------------------------
Keep the window on top to control your robot
Moving around:
        w     
   a    s    d

lshift/lctrl : increase/decrease max speeds by 10%
space key : force stop
anything else : stop smoothly

C or close window to quit
"""

max_speed = 3
max_turn = 0.5
speed = .2
turn = 0.5

def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    
    pygame.init()
    rospy.init_node('robot_teleop')
    pub = rospy.Publisher('/tianracer/ackermann_cmd', AckermannDrive, queue_size=5)
    screen = pygame.display.set_mode((200,100))
    screen.fill((255, 255, 255))
    pygame.display.set_caption("keep me on top")
    x = 0
    th = 0
    status = 0
    count = 0
    acc = 0.1
    target_speed = 0
    target_turn = 0
    control_speed = 0
    control_turn = 0
    try:
        print(msg)
        print(vels(speed,turn))
        while not rospy.is_shutdown():
            x = 0
            th = 0
            pygame.time.delay(42)
            key_list = pygame.key.get_pressed()
            pygame.event.pump()
            if key_list[pygame.K_w] or key_list[pygame.K_KP8]:
                if not (key_list[pygame.K_KP2] or key_list[pygame.K_s]):
                    x = 1
                else:
                    x = 0
                count = 0
            if key_list[pygame.K_s] or key_list[pygame.K_KP2]:
                if not (key_list[pygame.K_KP8] or key_list[pygame.K_w]):
                    x = -1
                else:
                    x = 0
                count = 0
            if key_list[pygame.K_a] or key_list[pygame.K_KP4]:
                if not (key_list[pygame.K_KP6] or key_list[pygame.K_d]):
                    th = 1
                else:
                    th = 0
                count = 0
            if key_list[pygame.K_d] or key_list[pygame.K_KP6]:
                if not (key_list[pygame.K_KP4] or key_list[pygame.K_a]):
                    th = -1
                else:
                    th = 0
                count = 0
            if key_list[pygame.K_SPACE] or key_list[pygame.K_KP5]:
                x = 0
                th = 0
                control_speed = 0
                control_turn = 0
            if key_list[pygame.K_LSHIFT]:
                speed = speed * 1.1
                turn = turn * 1.1
                count = 0
                count = 0

                print(vels(speed,turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            if key_list[pygame.K_LCTRL]:
                speed = speed * 0.9
                turn = turn * 0.9
                count = 0
                count = 0

                print(vels(speed,turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            # count = count + 1
            # if count > 40:
            #     x = 0
            #     th = 0
            if (key_list[pygame.K_c]):
                break

            # 目标速度=速度值*方向值
            target_speed = speed * x
            target_turn = turn * th

            # 速度限位，防止速度增减过快
            #if target_speed > control_speed:
            #    control_speed = min( target_speed, control_speed + 0.02 )
            #elif target_speed < control_speed:
            #    control_speed = max( target_speed, control_speed - 0.02 )
            #else:
            #    control_speed = target_speed

            #if target_turn > control_turn:
            #    control_turn = min( target_turn, control_turn + 0.1 )
            #elif target_turn < control_turn:
            #    control_turn = max( target_turn, control_turn - 0.1 )
            #else:
            #    control_turn = target_turn

	    if speed > max_speed:
		control_speed = max_speed * x
	    else:
		control_speed = target_speed

	    if turn > 0.5:
		control_turn = 0.5 * th
	    else:
		control_turn = target_turn
            # 创建并发布twist消息
            ackermann = AckermannDrive()
            ackermann.speed = control_speed; 
            ackermann.steering_angle = control_turn; 

            pub.publish(ackermann)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()
    except Exception as e:
        print(e)

    finally:
	ackermann = AckermannDrive()
	ackermann.speed = 0; 
	ackermann.steering_angle = 0;
        pub.publish(ackermann)

