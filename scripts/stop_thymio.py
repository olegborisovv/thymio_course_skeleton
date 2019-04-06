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

# a handy tool to convert orientations
from tf.transformations import euler_from_quaternion, quaternion_from_euler

rospy.init_node('basic_thymio_controller', anonymous=True)


class Thymio:
    def __init__(self, name):
        self.name = name
        self.velocity_publisher = rospy.Publisher('/%s/cmd_vel' % name, Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/%s/odom' % name, Odometry, self.update_odometry)

        self.current_pose = None
        self.current_twist = None
        self.rate = rospy.Rate(10)
        self.interrupt = lambda: False

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

    # def cmd_vel_duration(self, x, angle, duration):
    #     t0 = rospy.get_time()
    #     while True:
    #         self.cmd_vel(x, angle)
    #         if rospy.get_time() - t0 >= duration:
    #             self.cmd_vel(0.0, 0.0)
    #             break
    #         rospy.sleep(0.01)

    def has_pose(self):
        return self.current_pose is not None

    def wait_pose(self, timeout=3.0):
        rate = rospy.Rate(10)
        t0 = rospy.get_time()
        while (rospy.get_time() - t0) < timeout:
            if self.has_pose():
                return True
            rate.sleep()

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

    def lookpoint(self, x, y, tolerance=0.001):
        print("%s lookpoint %f,%f. now I'm in %f,%f " % (self.name, x, y, self.pose.x, self.pose.y))
        controller = PID(-4.0, 0.0, 0.0, 'lookpoint_pid')
        vel_msg = Twist()
        vect_y = y - self.pose.y
        vect_x = x - self.pose.x
        des_theta = np.arctan2(vect_y, vect_x)
        print ("des_theta %f" % (des_theta))
        t0 = rospy.get_time()
        print("des_theta,theta,error")
        while True:
            if self.interrupt():
                return
            e = angle_difference(self.pose.theta, des_theta)
            # print (des_theta, self.pose.theta, e)
            if np.abs(e) < tolerance:
                break

            t1 = rospy.get_time()
            dt = t1 - t0
            s = controller.step(e, dt)
            vel_msg.angular.z = s
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
            t0 = t1
        self.stop()

        print("%s lookpoint %f,%f. now I'm in %f,%f " % (self.name, x, y, self.pose.x, self.pose.y))

    def stop(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

    def follow(self, turtle, tolerance=1.0):
        print("%s follow %s" % (self.name, turtle.name))
        angle_ctrl = PID(-4.0, 0.0, 0.0, "pid_follow_angle").load()
        distance_ctrl = PID(2.5, 0.6, 1.0, "pid_follow_distance").load()
        vel_msg = Twist()
        t0 = rospy.get_time()
        print("angle_error,theta_vel")
        while True:
            if self.interrupt():
                return
            angle_ctrl.load()
            distance_ctrl.load()
            x = turtle.pose.x
            y = turtle.pose.y
            vect_y = y - self.pose.y
            vect_x = x - self.pose.x
            des_theta = np.arctan2(vect_y, vect_x)
            # angle_error =  self.pose.theta - des_theta
            angle_error = angle_difference(self.pose.theta, des_theta)
            distance_error = np.sqrt(pow(vect_x, 2) + pow(vect_y, 2))

            if distance_error < tolerance:
                break

            t1 = rospy.get_time()
            dt = t1 - t0
            if np.abs(angle_error) < (np.pi / 4):
                x_vel = distance_ctrl.step(distance_error, dt)
                vel_msg.linear.x = x_vel
            else:
                vel_msg.linear.x = 0
                distance_ctrl.reset()
            vel_msg.angular.z = angle_ctrl.step(angle_error, dt)
            print (angle_error, vel_msg.angular.z)
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
            t0 = t1
        self.stop()
        print("%s goto %f,%f. now I'm in %f,%f " % (self.name, x, y, self.pose.x, self.pose.y))

    def goto(self, x, y, tolerance=0.03):
        print("%s goto %f,%f. now I'm in %f,%f " % (self.name, x, y, self.pose.x, self.pose.y))
        angle_ctrl = PID(-4.0, 0.0, 0.0, "pid_goto_angle").load()
        distance_ctrl = PID(2.5, 0.6, 1.0, "pid_goto_dist").load()
        vel_msg = Twist()
        t0 = rospy.get_time()
        print("angle_error,theta_vel")
        while True:
            if self.interrupt():
                return
            angle_ctrl.load()
            distance_ctrl.load()
            vect_y = y - self.pose.y
            vect_x = x - self.pose.x
            des_theta = np.arctan2(vect_y, vect_x)
            # angle_error =  self.pose.theta - des_theta
            angle_error = angle_difference(self.pose.theta, des_theta)
            distance_error = np.sqrt(pow(vect_x, 2) + pow(vect_y, 2))

            if distance_error < tolerance:
                break

            t1 = rospy.get_time()
            dt = t1 - t0
            if np.abs(angle_error) < (np.pi / 4):
                x_vel = distance_ctrl.step(distance_error, dt)
                vel_msg.linear.x = x_vel
            else:
                vel_msg.linear.x = 0
                distance_ctrl.reset()
            vel_msg.angular.z = angle_ctrl.step(angle_error, dt)
            # print (angle_error, vel_msg.angular.z)
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
            t0 = t1
        self.stop()
        print("%s goto %f,%f. now I'm in %f,%f " % (self.name, x, y, self.pose.x, self.pose.y))

    def process_command(self, line):
        parts = line.split(',')
        cmd = parts[0]
        args = parts[1:]

        if cmd == 'trail':
            self.cmd_trail(args)
        elif cmd == 'lookpoint':
            self.cmd_lookpoint(args)
        elif cmd == 'goto':
            self.cmd_goto(args)
        else:
            print('command %s not recognized' % cmd)

    def cmd_trail(self, args):
        if len(args) != 1:
            print ("args: on")
            print ("args: off")
            return
        if args[0] == 'off':
            off = 1
        else:
            off = 0

        service_name = '/%s/set_pen' % self.name
        rospy.ServiceProxy(service_name, turtlesim.srv.SetPen)(200, 200, 0, 3, off)

    def cmd_lookpoint(self, args):
        if len(args) != 2:
            print ("args: x,y")
            return

        if not self.wait_pose():
            print ("timeout waiting pose")
        self.lookpoint(float(args[0]), float(args[1]))

    def cmd_goto(self, args):
        if len(args) != 2:
            print ("args: x,y")
            return

        if not self.wait_pose():
            print ("timeout waiting pose")
        self.goto(float(args[0]), float(args[1]))

    def set_interruptor(self, interruptor):
        if interruptor:
            self.interrupt = interruptor
        else:
            self.interrupt = lambda: False


class BasicThymio:

    def __init__(self, thymio_name):
        self.thymio_name = thymio_name
        self.velocity_publisher = rospy.Publisher(self.thymio_name + '/cmd_vel',
                                                  Twist, queue_size=10)

        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber(self.thymio_name + '/odom',
                                                Odometry, self.update_state)

        self.current_pose = None
        self.current_twist = None
        # publish at this rate
        self.rate = rospy.Rate(10)

    def thymio_state_service_request(self, position, orientation):
        """Request the service (set thymio state values) exposed by
        the simulated thymio. A teleportation tool, by default in gazebo world frame.
        Be aware, this does not mean a reset (e.g. odometry values)."""
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            model_state = ModelState()
            model_state.model_name = self.thymio_name
            model_state.reference_frame = ''  # the frame for the pose information
            model_state.pose.position.x = position[0]
            model_state.pose.position.y = position[1]
            model_state.pose.position.z = position[2]
            qto = quaternion_from_euler(orientation[0], orientation[0], orientation[0], axes='sxyz')
            model_state.pose.orientation.x = qto[0]
            model_state.pose.orientation.y = qto[1]
            model_state.pose.orientation.z = qto[2]
            model_state.pose.orientation.w = qto[3]
            # a Twist can also be set but not recomended to do it in a service
            gms = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            response = gms(model_state)
            return response
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def update_state(self, data):
        """A new Odometry message has arrived. See Odometry msg definition."""
        # Note: Odmetry message also provides covariance
        self.current_pose = data.pose.pose
        self.current_twist = data.twist.twist
        quat = (
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w)
        (roll, pitch, yaw) = euler_from_quaternion(quat)
        # rospy.loginfo("State from Odom: (%.5f, %.5f, %.5f) " % (self.current_pose.position.x, self.current_pose.position.y, yaw))

    def has_pose(self):
        return self.current_pose is not None

    def wait_pose(self, timeout=3.0):
        rate = rospy.Rate(10)
        t0 = rospy.get_time()
        while (rospy.get_time() - t0) < timeout:
            if self.has_pose():
                return True
            rate.sleep()

        return self.has_pose()


def usage():
    return "Wrong number of parameters. basic_move.py [thymio_name]"


if __name__ == '__main__':

    rate = rospy.Rate(10)

    thymio = Thymio("thymio10")

    if not thymio.wait_pose():
        print("timeout for pose")

    for i in range(10):
        thymio.cmd_vel(0.0, 0.0)
        rate.sleep()
