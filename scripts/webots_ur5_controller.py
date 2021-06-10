#!/usr/bin/env python
"""universal_robot_ros controller."""

import argparse
import rospy
import time as timee

from controller import Robot, TouchSensor, Camera, RangeFinder
from joint_state_publisher import JointStatePublisher
from trajectory_follower import TrajectoryFollower
from trajectory_follower_gripper import TrajectoryFollowerGripper
from rosgraph_msgs.msg import Clock
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
import numpy as np

import os
os.environ["WEBOTS_ROBOT_NAME"] = "UR5e"

parser = argparse.ArgumentParser()
parser.add_argument('--node-name', dest='nodeName', default='ur_driver', help='Specifies the name of the node.')
arguments, unknown = parser.parse_known_args()

class TimeIt:
	def __init__(self, s):
		self.s = s
		self.t0 = None
		self.t1 = None
		self.print_output = False

	def __enter__(self):
		self.t0 = timee.time()

	def __exit__(self, t, value, traceback):
		self.t1 = timee.time()
		rospy.logwarn('%s: %5s s' % (self.s, self.t1 - self.t0))


def main():
	jointPrefix = rospy.get_param('prefix', '')
	if jointPrefix:
		print('Setting prefix to %s' % jointPrefix)

	nodeName = arguments.nodeName + '/' if arguments.nodeName != 'ur_driver' else ''

	robot = Robot()
	jointStatePublisher = JointStatePublisher(robot, jointPrefix, nodeName)
	trajectoryFollower = TrajectoryFollower(robot, jointStatePublisher, jointPrefix, nodeName)
	trajectoryFollower.start()

	# MODIFICADO
	gripperjointPrefix = rospy.get_param('prefix', '')
	gripperStatePublisher = JointStatePublisher(robot, gripperjointPrefix, nodeName)

	sensors = ['bumper1', 'bumper2']
	touch_sensors = [TouchSensor(bumper) for bumper in sensors]
	for touch_sensor in touch_sensors:
		touch_sensor.enable(4)

	GrippertrajectoryFollower = TrajectoryFollowerGripper(robot, gripperStatePublisher, gripperjointPrefix, touch_sensors)
	GrippertrajectoryFollower.start()

	# we want to use simulation time for ROS
	clockPublisher = rospy.Publisher('clock', Clock, queue_size=1)
	if not rospy.get_param('use_sim_time', False):
		rospy.logwarn('use_sim_time is not set!')

	timestep = int(robot.getBasicTimeStep())

	while robot.step(timestep) != -1 and not rospy.is_shutdown():
		jointStatePublisher.publish()
		trajectoryFollower.update()
		gripperStatePublisher.publish()
		GrippertrajectoryFollower.update_gripper()
		# pulish simulation clock
		msg = Clock()
		time = robot.getTime()
		msg.clock.secs = int(time)
		# round prevents precision issues that can cause problems with ROS timers
		msg.clock.nsecs = round(1000 * (time - msg.clock.secs)) * 1.0e+6
		clockPublisher.publish(msg)


if __name__ == "__main__":
	# log_level=rospy.FATAL, 
	rospy.init_node(arguments.nodeName, log_level=rospy.INFO, disable_signals=True)
	main()
	