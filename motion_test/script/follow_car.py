#!/usr/bin/env python
import roslib
import rospy
import socket
import time
import math
import numpy
from ackermann_msgs.msg import AckermannDrive

test_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
target_x = 1
target_y = 0
max_angle = 0.5    # the max angle is 0.5
max_speed = 1    # the max speed is 1
k_p_speed = 1.5
k_p_angle = 0.5
v0 = 0
ackermann = AckermannDrive()
ackermann.speed = 0.0
ackermann.steering_angle = 0.0
ackermann_cmd_pub = rospy.Publisher('/tianracer/ackermann_cmd', AckermannDrive, queue_size=5)
s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
s.connect(("127.0.0.1", 6666))

def get_goal_position():
    """
    docstring
    """
    msgs = s.recv(1000).replace('(', '').replace(')', '').replace(' ', '').split(',') 
    x = float(msgs[4])
    y = float(msgs[2])
    return x, y

def get_v0(history_distance, T):
    """
    docstring
    """
    if -1 in history_distance:
        return 0
    else:
        v_first_car = (history_distance[-1] - history_distance[0]) * T + ackermann.speed
        return v_first_car

def is_zero(test_number, max_number, tolerance):
    if abs(test_number)/max_number < tolerance:
        return True
    else:
        return False

def get_command(x, y, v0):
    """
    position control
    """
    dx = x - target_x
    dy = y - target_y
    if dx > 0:
        d_theda = -y / x
    else:
	    d_theda = y / x
    if d_theda > max_angle:
        angle = max_angle
    elif d_theda < -max_angle:
        angle = -max_angle
    else:
        angle = d_theda * k_p_angle
    # the max angle is 0.5
    d_speed = dx * k_p_speed + v0
    #d_speed = (math.sqrt(x*x+y*y)-1)*k_p_speed + v0
    if d_speed > max_speed:
        speed = max_speed
    elif d_speed < -max_speed:
        speed = -max_speed
    else:
        speed = d_speed

    if (not is_zero(angle, max_angle, 0.1)) and is_zero(speed, max_speed, 0.05):
        speed = -0.5
    # the max speed is 1
    return speed, angle

if __name__ == "__main__":
    rospy.init_node('ros_talker')
    cnt = 0
    T = 20
    r = rospy.Rate(T)
    f = open('/home/tianbot/tianbot_ws/src/tianracer/tianracer_test/scripts/test.txt','a')
    f.write('\n%s' % test_time)
    #history_distance = [-1,-1,-1]
    try:
        #f.write('\n%s' % test_time)
        while not rospy.is_shutdown():
            x1, y1 = get_goal_position()
            # r.sleep()
            x2, y2 = get_goal_position()
            x = (x1 + x2) / 2
            y = (y1 + y2) / 2
            #history_distance.append(x)
            #history_distance.pop(0)
            #v0 = get_v0(history_distance, T)
            speed, angle = get_command(x, y, v0)
            ackermann.speed = speed
            ackermann.steering_angle = angle
	        #rospy.loginfo("x, y, speed, angle: %s, %s, %s, %s" % (x, y, speed, angle))
            ackermann_cmd_pub.publish(ackermann)
            cnt += 1
            print(cnt,x,y,speed,angle)
            f.write('\nx, y, speed, angle: %s, %s, %s, %s' % (x, y, speed, angle))
            # r.sleep()
            

    except Exception as e:
        print(e)

    finally:
        ackermann = AckermannDrive()
        ackermann.speed = 0.0
        ackermann.steering_angle = 0.0
        ackermann_cmd_pub.publish(ackermann)
        s.close()
        f.close()
