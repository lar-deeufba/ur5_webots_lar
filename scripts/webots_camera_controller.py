#!/usr/bin/env python
"""camera controller."""

import argparse
import rospy
import time as timee

from controller import Robot, Camera, RangeFinder
from rosgraph_msgs.msg import Clock
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument('--node-name', dest='nodeName', default='camera_node', help='Specifies the name of the node.')
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

	nodeName = arguments.nodeName + '/' if arguments.nodeName != 'camera_node' else ''

	robot = Robot()

	# we want to use simulation time for ROS
	clockPublisher = rospy.Publisher('clock', Clock, queue_size=1)
	if not rospy.get_param('use_sim_time', False):
		rospy.logwarn('use_sim_time is not set!')

	timestep = 100
	br = CvBridge()

	##################
	# RGB CAMERA  
	##################
	camera = Camera("camera")
	camera.enable(timestep)
	
	color_camera_info_param = rospy.get_param('/color_camera_info')
	color_camera_info = CameraInfo()
	color_camera_info.header.frame_id = color_camera_info_param['frame_id']
	color_camera_info.height = color_camera_info_param['height']
	color_camera_info.width = color_camera_info_param['width']
	color_camera_info.distortion_model = color_camera_info_param['distortion_model']
	color_camera_info.D = color_camera_info_param['D']
	color_camera_info.K = color_camera_info_param['K']
	color_camera_info.R = color_camera_info_param['R']
	color_camera_info.P = color_camera_info_param['P']
	color_camera_info.binning_x = color_camera_info_param['binning_x']
	color_camera_info.binning_y = color_camera_info_param['binning_y']
	color_camera_info.roi.x_offset = color_camera_info_param['roi']['x_offset']
	color_camera_info.roi.y_offset = color_camera_info_param['roi']['y_offset']
	color_camera_info.roi.height = color_camera_info_param['roi']['height']
	color_camera_info.roi.width = color_camera_info_param['roi']['width']
	color_camera_info.roi.do_rectify = color_camera_info_param['roi']['do_rectify']

	color_sensor_info_pub = rospy.Publisher('webots/ur5/color_camera_info', CameraInfo, queue_size=10)
	color_pub = rospy.Publisher('webots/ur5/color_camera', Image,queue_size=10)

	##################
	# DEPTH SENSOR
	##################
	depth_sensor = RangeFinder("depth-sensor")
	depth_sensor.enable(timestep)

	depth_camera_info_param = rospy.get_param('/depth_camera_info')
	depth_camera_info = CameraInfo()
	depth_camera_info.header.frame_id = depth_camera_info_param['frame_id']
	depth_camera_info.height = depth_camera_info_param['height']
	depth_camera_info.width = depth_camera_info_param['width']
	depth_camera_info.distortion_model = depth_camera_info_param['distortion_model']
	depth_camera_info.D = depth_camera_info_param['D']
	depth_camera_info.K = depth_camera_info_param['K']
	depth_camera_info.R = depth_camera_info_param['R']
	depth_camera_info.P = depth_camera_info_param['P']
	depth_camera_info.binning_x = depth_camera_info_param['binning_x']
	depth_camera_info.binning_y = depth_camera_info_param['binning_y']
	depth_camera_info.roi.x_offset = depth_camera_info_param['roi']['x_offset']
	depth_camera_info.roi.y_offset = depth_camera_info_param['roi']['y_offset']
	depth_camera_info.roi.height = depth_camera_info_param['roi']['height']
	depth_camera_info.roi.width = depth_camera_info_param['roi']['width']
	depth_camera_info.roi.do_rectify = depth_camera_info_param['roi']['do_rectify']

	depth_sensor_info_pub = rospy.Publisher('webots/ur5/depth_camera_info', CameraInfo, queue_size=10)
	depth_pub = rospy.Publisher('webots/ur5/depth_camera', Image, queue_size=10)

	while robot.step(timestep) != -1 and not rospy.is_shutdown():
		msg = Clock()
		time = robot.getTime()
		msg.clock.secs = int(time)
		# round prevents precision issues that can cause problems with ROS timers
		msg.clock.nsecs = round(1000 * (time - msg.clock.secs)) * 1.0e+6
		clockPublisher.publish(msg)

		# Color Camera Info
		color_camera_info.header.stamp.secs = msg.clock.secs
		color_camera_info.header.stamp.nsecs = msg.clock.nsecs
		color_sensor_info_pub.publish(color_camera_info)

		# Read and publish Camera
		cameraData = camera.getImage()
		image = np.frombuffer(cameraData, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))
		image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
		image_msg = br.cv2_to_imgmsg(image,"bgr8")
		image_msg.header = color_camera_info.header
		color_pub.publish(image_msg)

		# Depth Camera Info
		depth_camera_info.header.stamp.secs = msg.clock.secs
		depth_camera_info.header.stamp.nsecs = msg.clock.nsecs
		depth_sensor_info_pub.publish(depth_camera_info)

		# Read and publish depth cam
		depth_cam = depth_sensor.getRangeImageArray()
		depth_cam = np.asarray(depth_cam)
		depth_cam = depth_cam*1000	
		depth_cam = depth_cam.astype(np.uint16)	
		depth_cam = cv2.rotate(depth_cam, cv2.ROTATE_90_CLOCKWISE)	
		depth_cam = cv2.flip(depth_cam, 1)	
		depth_cropped_imgmsg = br.cv2_to_imgmsg(depth_cam)
		depth_cropped_imgmsg.header = depth_camera_info.header
		depth_pub.publish(depth_cropped_imgmsg)

if __name__ == "__main__":
	rospy.init_node(arguments.nodeName, log_level=rospy.FATAL, disable_signals=True)
	
	main()