#!/usr/bin/env python

import roslib
import rospy
import socket
import time
import math
import numpy as np
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import PoseStamped

test_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
target_x = 1
target_y = 0
target_theta = 0
d_T = 0.05
max_angle = 0.5    # the max angle is 0.5
max_speed = 1    # the max speed is 1
k_p_speed = 1.5
k_p_angle = 0.5
k_theta_to_y = 1
k_i_speed = 1.5
k_i_angle = 0.5
error_i = np.array([0, 0, 0])
v0 = 0
x_current = 0
y_current = 0
theta_current = 0


def get_current_position(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.pose.position.x)
    x_current = data.pose.position.x
    y_current = data.pose.position.y
    theta_current = 0


ackermann = AckermannDrive()
ackermann.speed = 0.0
ackermann.steering_angle = 0.0
ackermann_cmd_pub = rospy.Publisher(
    '/tianracer/ackermann_cmd', AckermannDrive, queue_size=5)
pose_sub = rospy.Subscriber(
    'vrpn_client_node/vehicle/pose', PoseStamped, get_current_position)


def run_as_circle(x, y):
    v = 0.0
    steering_angle = 0.0
    # need to be completed
    return v, steering_angle


def follow_straight_line(start_point, end_point):
    v_prefer = max_speed/2.0
    distance = math.sqrt(
        (end_point[0]-start_point[0])**2+(end_point[1]-start_point[1])**2)
    T = distance/v_prefer
    number_of_path = int(T/d_T)
    x_points = np.linspace(start_point[0], end_point[0], number_of_path+1)
    y_points = np.linspace(start_point[1], end_point[1], number_of_path+1)
    theta_points = np.zeros(number_of_path+1)
    theta_points[0] = start_point[2]
    theta_points[number_of_path] = end_point[2]
    for i in range(number_of_path-1):
        theta_points[i+1] = (theta_points[i]+math.atan2((y_points[i+1] -
                             y_points[i]), (x_points[i+1]-x_points[i])))/2
    return x_points, y_points, theta_points


def PID(current_target_x, current_target_y, current_target_theta):
    # Vertical control
    R = np.array([[math.cos(theta_current), math.sin(theta_current), 0],
                 [-math.sin(theta_current), math.cos(theta_current), 0], [0, 0, 1]])
    world_frame_point = np.array(
        [current_target_x-x_current, current_target_y-y_current, current_target_theta-theta_current])
    car_frame_point = np.dot(R, world_frame_point)
    car_frame_error = np.dot(R, error_i)
    error_p_vertical = car_frame_point[0]
    error_i_vertical = car_frame_error[0]
    v = error_p_vertical*k_p_speed + error_i_vertical*k_i_speed

    # Lateral control
    error_p_lateral = car_frame_point[1] + k_theta_to_y*(car_frame_point[2])
    error_i_lateral = car_frame_error[1] + k_theta_to_y*car_frame_error[2]
    steering_angle = error_p_lateral*k_p_angle + error_i_lateral*k_i_angle
    return v, steering_angle


def publish_command(v, steering_angle):
    ackermann.speed = v
    ackermann.steering_angle = steering_angle
    ackermann_cmd_pub.publish(ackermann)


def main():
    rospy.init_node('test_demo_node')
    cnt = 0
    T = 20
    r = rospy.Rate(T)
    # f = open('/home/tianbot/tianbot_ws/src/tianracer/tianracer_test/scripts/test.txt','a')
    # f.write('\n%s' % test_time)
    start_point = np.array([x_current, y_current, theta_current])
    end_point = np.array([target_x, target_y, target_theta])
    x_points, y_points, theta_points = follow_straight_line(
        start_point, end_point)
    current_target_x = x_points[cnt]
    current_target_y = y_points[cnt]
    current_target_theta = theta_points[cnt]
    cnt = cnt + 1
    #history_distance = [-1,-1,-1]

    try:
        #f.write('\n%s' % test_time)
        while not rospy.is_shutdown():
            x_current_change = x_current
            y_current_change = y_current
            theta_current_change = theta_current
            error_i[0] = error_i[0] + current_target_x - x_current_change
            error_i[1] = error_i[1] + current_target_y - y_current_change
            error_i[2] = error_i[2] + \
                current_target_theta - theta_current_change
            current_target_x = x_points[cnt]
            current_target_y = y_points[cnt]
            current_target_theta = theta_points[cnt]
            x_points, y_points, theta_points = follow_stright_line(
                start_point, end_point)
            v, steering_angle = PID(
                current_target_x, current_target_y, current_target_theta)
            publish_command(v, steering_angle)
            # history_distance.append(x)
            # history_distance.pop(0)
            #v0 = get_v0(history_distance, T)

            #rospy.loginfo("x, y, speed, angle: %s, %s, %s, %s" % (x, y, speed, angle))
            cnt += 1
            print(cnt, x_current, y_current, v, steering_angle)
            # f.write('\nx, y, speed, angle: %s, %s, %s, %s' % (x, y, speed, angle))
            # r.sleep()
            rospy.spin()

    except Exception as e:
        print(e)

    finally:
        ackermann = AckermannDrive()
        ackermann.speed = 0.0
        ackermann.steering_angle = 0.0
        ackermann_cmd_pub.publish(ackermann)
        # f.close()


if __name__ == '__main__':
    main()
