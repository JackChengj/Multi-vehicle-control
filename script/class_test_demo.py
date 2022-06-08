#!/usr/bin/env python

import roslib
import rospy
import socket
import time
import math
import threading
import numpy as np
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import PoseStamped


class TestDemo:
    def __init__(self):
        self.sub = rospy.Subscriber(
            'vrpn_client_node/vehicle/pose', PoseStamped, self.callback)
        self.pub = rospy.Publisher(
            '/tianracer/ackermann_cmd', AckermannDrive, queue_size=5)
        self.ackermann_msgs = AckermannDrive()
        self.ackermann_msgs.speed = 0
        self.ackermann_msgs.steering_angle = 0
        self.d_T = 0.05
        self.max_angle = 0.5    # the max angle is 0.5
        self.max_speed = 1    # the max speed is 1
        self.k_p_speed = 1.5
        self.k_p_angle = 0.5
        self.k_theta_to_y = 1
        self.k_i_speed = 1.5
        self.k_i_angle = 0.5
        self.error_i = np.array([0, 0, 0])
        self.v0 = 0
        self.target_x = 1
        self.target_y = 0
        self.target_theta = 0
        self.x_current = 0
        self.y_current = 0
        self.theta_current = 0
        self.buffer_A = PoseStamped()
        self.buffer_B = PoseStamped()
        self.buffer_C = PoseStamped()
        self.x_points = []
        self.y_points = []
        self.theta_points = []
        self.cnt = 0
        self.follow_straight_line()

    def callback(self, data):
        rospy.loginfo(rospy.get_caller_id() +
                      "I heard %s", data.pose.position.x)
        if data.header.seq % 3 == 0:
            self.buffer_A = data
        elif data.header.seq % 3 == 1:
            self.buffer_B = data
        else:
            self.buffer_C = data

    def follow_straight_line(self):
        v_prefer = self.max_speed/2.0
        distance = math.sqrt(
            (self.target_x-self.x_current)**2+(self.target_y-self.y_current)**2)
        T = distance/v_prefer
        number_of_path = int(T/self.d_T)
        self.x_points = np.linspace(
            self.x_current, self.target_x, number_of_path+1)
        self.y_points = np.linspace(
            self.y_current, self.target_y, number_of_path+1)
        self.theta_points = np.zeros(number_of_path+1)
        self.theta_points[0] = self.theta_current
        self.theta_points[number_of_path] = self.target_theta
        for i in range(number_of_path-1):
            self.theta_points[i+1] = (self.theta_points[i]+math.atan2(
                (self.y_points[i+1]-self.y_points[i]), (self.x_points[i+1]-self.x_points[i])))/2

    def controller(self):
        if self.cnt < len(self.x_points):
            self.get_current_position()
            v, steering_angle = self.PID(
                self.x_points[self.cnt], self.y_points[self.cnt], self.theta_points[self.cnt])
            self.ackermann_msgs.speed = v
            self.ackermann_msgs.steering_angle = steering_angle
            self.pub.publish(self.ackermann_msgs)
            rospy.loginfo("publishing {}".format(v))
            self.cnt = self.cnt+1
        else:
            rospy.loginfo("The vehicle arrived the end.")

    def get_current_position(self):
        self.x_current = self.buffer_A.pose.position.x
        self.y_current = self.buffer_A.pose.position.y
        self.theta_current = math.atan2(self.buffer_A.pose.position.y-(self.buffer_B.pose.position.y+self.buffer_C.pose.position.y)/2,
                                        self.buffer_A.pose.position.x-(self.buffer_B.pose.position.x+self.buffer_C.pose.position.x)/2)

    def PID(self, current_target_x, current_target_y, current_target_theta):
        # Vertical control
        R = np.array([[math.cos(self.theta_current), math.sin(self.theta_current), 0],
                     [-math.sin(self.theta_current), math.cos(self.theta_current), 0], [0, 0, 1]])
        world_frame_point = np.array(
            [current_target_x-self.x_current, current_target_y-self.y_current, current_target_theta-self.theta_current])
        car_frame_point = np.dot(R, world_frame_point)
        car_frame_error = np.dot(R, self.error_i)
        error_p_vertical = car_frame_point[0]
        error_i_vertical = car_frame_error[0]
        v = error_p_vertical*self.k_p_speed + error_i_vertical*self.k_i_speed

        # Lateral control
        error_p_lateral = car_frame_point[1] + \
            self.k_theta_to_y*(car_frame_point[2])
        error_i_lateral = car_frame_error[1] + \
            self.k_theta_to_y*car_frame_error[2]
        steering_angle = error_p_lateral*self.k_p_angle + error_i_lateral*self.k_i_angle
        return v, steering_angle

    # def thread_job(self):
    #     rospy.spin()

    def main(self):
        try:
            # ROS Init
            rospy.init_node('test_demo_node', anonymous=True)
            # add_thread = threading.Thread(target=self.thread_job)
            # add_thread.start()

            while not rospy.is_shutdown():
                self.get_current_position()
                self.controller()
                time.sleep(self.d_T)

            rospy.spin()

        except Exception as e:
            print(e)

        finally:
            ackermann = AckermannDrive()
            ackermann.speed = 0.0
            ackermann.steering_angle = 0.0
            self.pub.publish(ackermann)


if __name__ == "__main__":
    demo = TestDemo()
    demo.main()
