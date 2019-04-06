#!/usr/bin/env python
import rospy
import sys

import numpy as np
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from math import cos, sin, asin, tan, atan2, sqrt
# msgs and srv for working with the set_model_service
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
import std_srvs.srv

import os
import random

from sensor_msgs.msg import Range

# a handy tool to convert orientations
from tf.transformations import euler_from_quaternion, quaternion_from_euler

rospy.init_node('basic_thymio_controller', anonymous=True)


class BasicSensors:
    def __init__(self, thymio):
        self.thymio = thymio
        self.left = None
        self.right = None

    def facinig_wall(self, tolerance=10 ** (-4)):
        return abs(self.left - self.right) < tolerance


class BackSensors(BasicSensors):
    def __init__(self, thymio):
        BasicSensors.__init__(self, thymio)
        self.left_subs = rospy.Subscriber('/%s/proximity/rear_left' % self.thymio.name, Range, self.general_handler)
        self.right_subs = rospy.Subscriber('/%s/proximity/rear_right' % self.thymio.name, Range, self.general_handler)

    def general_handler(self, data):
        topic = str(data._connection_header['topic'])
        if topic.endswith('/rear_left'):
            self.left = data.range
        elif topic.endswith('/rear_right'):
            self.right = data.range

    def has_reading(self):
        return len([x for x in [self.left, self.right] if x is not None]) == 2


class FrontSensors(BasicSensors):
    def __init__(self, thymio):
        self.center = None
        BasicSensors.__init__(self, thymio)

        self.left_subs = rospy.Subscriber('/%s/proximity/left' % self.thymio.name, Range, self.general_handler)
        self.right_subs = rospy.Subscriber('/%s/proximity/right' % self.thymio.name, Range, self.general_handler)
        self.right_subs = rospy.Subscriber('/%s/proximity/center' % self.thymio.name, Range, self.general_handler)

    def general_handler(self, data):
        topic = str(data._connection_header['topic'])
        if topic.endswith('/left'):
            self.left = data.range
        elif topic.endswith('/right'):
            self.right = data.range
        elif topic.endswith('/center'):
            self.center = data.range

    def has_reading(self):
        return len([x for x in [self.left, self.right, self.center] if x is not None]) == 3


class Thymio:
    def __init__(self, name):
        self.name = name
        self.velocity_publisher = rospy.Publisher('/%s/cmd_vel' % name, Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/%s/odom' % name, Odometry, self.update_odometry)

        self.back_sensors = BackSensors(self)
        self.front_sensors = FrontSensors(self)

        self.current_pose = None
        self.current_twist = None
        self.rate = rospy.Rate(10)
        self.interrupt = lambda: False

    def is_wall_detected(self, distance_to_wall):
        measures = [self.front_sensors.left, self.front_sensors.center, self.front_sensors.right]
        for measure in measures:
            if measure < distance_to_wall:
                return True
        return False

    # Proximity Center

    def update_odometry(self, data):
        # Note: Odmetry message also provides covariance
        self.current_pose = data.pose.pose
        self.current_twist = data.twist.twist
        quat = (
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w)
        (roll, pitch, yaw) = euler_from_quaternion(quat)

    def free(self):
        self.pose_subscriber.unregister()

    def cmd_vel(self, x, angle):
        vel_msg = Twist()
        vel_msg.linear.x = x
        vel_msg.angular.z = angle

        self.velocity_publisher.publish(vel_msg)

    def has_pose(self):
        return self.current_pose is not None

    def wait_pose(self, timeout=3.0):
        rate = rospy.Rate(10)
        t0 = rospy.get_time()
        while (rospy.get_time() - t0) < timeout:
            if self.has_pose():
                return True
            rate.sleep()
        return False

    def wait_sensors(self, timeout=3.0):
        rate = rospy.Rate(10)
        t0 = rospy.get_time()
        while (rospy.get_time() - t0) < timeout:
            if self.front_sensors.has_reading() and self.back_sensors.has_reading():
                return True
            rate.sleep()

        return False

    def euclidean_distance(self, target_pose):
        """Euclidean distance between current pose and the goal."""
        pose = self.current_pose.position
        goal_pose = target_pose.position
        return sqrt(pow((goal_pose.x - pose.x), 2) +
                    pow((goal_pose.y - pose.y), 2))

    def linear_vel(self, goal_pose, constant=1.5):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        print("pose:", self.pose, "goal:", goal_pose)
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=6):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)


def main():
    rate = rospy.Rate(10)

    thymio = Thymio("thymio10")

    if not (thymio.wait_pose() and thymio.wait_sensors()):
        print("timeout for pose or sensors")

    print("Looking for the wall...")
    wall_detected = False
    for i in range(1000):
        thymio.cmd_vel(0.1, 0.0)

        wall_detected = thymio.is_wall_detected(distance_to_wall=0.05)
        if wall_detected:
            print ("wall detected")
            break

        rate.sleep()

    thymio.cmd_vel(0.0, 0.0)

    if not wall_detected:
        print("wall is not detected ")
        return

    print ("Rotating towards the wall")

    while True:
        if thymio.front_sensors.facinig_wall():
            print ("we are front facing the wall")
            break
        else:
            # if proportional_term is positive, we turn left
            # if negative - right
            proportional_term = thymio.front_sensors.right - thymio.front_sensors.left
            thymio.cmd_vel(0.0, proportional_term * 3)

        rate.sleep()

    print ("Fast rotating approximately front away the wall")
    for i in range(30):
        thymio.cmd_vel(0.0, np.pi / 3)
        rate.sleep()

    thymio.cmd_vel(0.0, 0.0)

    print ("Fine tuning the back facing")

    while True:
        if thymio.back_sensors.facinig_wall():
            print ("we are back facing the wall")
            break
        else:
            # if proportional_term is positive, we turn left
            # if negative - right
            proportional_term = thymio.back_sensors.right - thymio.back_sensors.left
            thymio.cmd_vel(0.0, proportional_term * 3)

        rate.sleep()

    thymio.cmd_vel(0.0, 0.0)
    print('diff between left and right rear sensors:', thymio.back_sensors.left - thymio.back_sensors.right)

    print ("going to 2 mt far from the wall")
    start_pose_from_the_wall = thymio.current_pose
    initial_wall_distance = thymio.back_sensors.right
    wanted_distance = 2.0 - initial_wall_distance

    distance = 0.0
    for i in range(20 * 10):
        thymio.cmd_vel(0.1, 0.0)
        p = thymio.current_pose
        distance = thymio.euclidean_distance(start_pose_from_the_wall)
        if distance > wanted_distance:
            print("we reach the final goal positin")
            break
        if i % 30 == 0:
            print("final estimated distance from the wall is ", distance + initial_wall_distance)
        rate.sleep()
    print("final estimated distance from the wall is ", distance + initial_wall_distance)

    print ("Fast rotating to approximately face the wall")
    print("It was not requested by the assignment but it is useful to do repeated tests run")
    for i in range(30):
        thymio.cmd_vel(0.0, np.pi / 3)
        rate.sleep()
    thymio.cmd_vel(0.0, 0.0)


if __name__ == '__main__':
    main()
