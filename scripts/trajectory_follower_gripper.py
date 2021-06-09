# Copyright 1996-2020 Cyberbotics Ltd.
# traj follow
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Implementation of the 'follow_joint_trajectory' ROS action."""

import actionlib
import copy
import math
import rospy

from control_msgs.msg import FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def trajectory_is_finite(trajectory):
	"""Check if trajectory contains infinite or NaN value."""
	for point in trajectory.points:
		for position in point.positions:
			if math.isinf(position) or math.isnan(position):
				return False
		for velocity in point.velocities:
			if math.isinf(velocity) or math.isnan(velocity):
				return False
	return True

def has_velocities(trajectory):
	"""Check that velocities are defined for this trajectory."""
	for point in trajectory.points:
		if len(point.velocities) != len(point.positions):
			return False
	return True

def within_tolerance(a_vec, b_vec, tol_vec):
	"""Check if two vectors are equals with a given tolerance."""
	for a, b, tol in zip(a_vec, b_vec, tol_vec):
		if abs(a - b) > tol:
			return False
	return True

def interp_cubic(p0, p1, t_abs):
	"""Perform a cubic interpolation between two trajectory points."""
	T = (p1.time_from_start - p0.time_from_start).to_sec()
	t = t_abs - p0.time_from_start.to_sec()
	q = [0] * 6
	qdot = [0] * 6
	qddot = [0] * 6
	for i in range(len(p0.positions)):
		a = p0.positions[i]
		b = p0.velocities[i]
		c = (-3 * p0.positions[i] + 3 * p1.positions[i] - 2 * T * p0.velocities[i] - T * p1.velocities[i]) / T**2
		d = (2 * p0.positions[i] - 2 * p1.positions[i] + T * p0.velocities[i] + T * p1.velocities[i]) / T**3

		q[i] = a + b * t + c * t**2 + d * t**3
		qdot[i] = b + 2 * c * t + 3 * d * t**2
		qddot[i] = 2 * c + 6 * d * t
	return JointTrajectoryPoint(positions=q, velocities=qdot, accelerations=qddot, time_from_start=rospy.Duration(t_abs))


def sample_trajectory(trajectory, t):
	"""Return (q, qdot, qddot) for sampling the JointTrajectory at time t,
	   the time t is the time since the trajectory was started."""
	# First point
	if t <= 0.0:
		return copy.deepcopy(trajectory.points[0])
	# Last point
	if t >= trajectory.points[-1].time_from_start.to_sec():
		return copy.deepcopy(trajectory.points[-1])
	# Finds the (middle) segment containing t
	i = 0
	while trajectory.points[i + 1].time_from_start.to_sec() < t:
		i += 1

	return interp_cubic(trajectory.points[i], trajectory.points[i + 1], t)


class TrajectoryFollowerGripper(object):
	"""Create and handle the action 'follow_joint_trajectory' server."""

	gripperjointNames = rospy.get_param("/robotiq_joint_name")

	def __init__(self, robot, jointStatePublisher, jointPrefix, TouchSensors):
		self.robot = robot
		self.gripperjointPrefix = jointPrefix
		self.gripperprefixedJointNames = [s + self.gripperjointPrefix for s in TrajectoryFollowerGripper.gripperjointNames]
		self.jointStatePublisher = jointStatePublisher # USADO?
		self.timestep = int(robot.getBasicTimeStep()) # OK
		self.grippermotors = [] # OK
		self.grippersensors = [] # OK
		
		self.TouchSensors = TouchSensors
		
		for name in TrajectoryFollowerGripper.gripperjointNames: # OK
			self.grippermotors.append(robot.getDevice(name)) # OK
			self.grippersensors.append(robot.getDevice(name + '_sensor')) # OK
			self.grippersensors[-1].enable(self.timestep) # OK
		
		self.joint_directions = [1, -1, 1, -1, -1, 1]
		self.gripper_pos_atual = [0] * len(TrajectoryFollowerGripper.gripperjointNames)
		self.goal_handle_gripper = None # OK
		self.last_point_sent_gripper = True # OK
		self.trajectory_gripper = None # OK
		self.joint_goal_tolerances_gripper = [0.05]*len(TrajectoryFollowerGripper.gripperjointNames) # OK
		self.gripperserver = actionlib.ActionServer("gripper_controller/follow_joint_trajectory",
											 FollowJointTrajectoryAction,
											 self.on_goal_gripper, self.on_cancel_gripper, auto_start=False)   
										  

	def start(self):
		"""Initialize and start the action server."""
		self.init_trajectory_gripper()
		self.gripperserver.start()
		print("The action server for this driver has been started")

	def init_trajectory_gripper(self):
		"""Initialize a new target trajectory."""
		state = self.jointStatePublisher.last_joint_states
		self.trajectory_gripper_t0 = self.robot.getTime()
		self.trajectory_gripper = JointTrajectory()
		self.trajectory_gripper.joint_names = self.gripperprefixedJointNames
		self.trajectory_gripper.points = [JointTrajectoryPoint(
			positions=self.gripper_pos_atual,
			velocities=[0] * len(TrajectoryFollowerGripper.gripperjointNames),
			accelerations=[0] * len(TrajectoryFollowerGripper.gripperjointNames),
			time_from_start=rospy.Duration(0.0))]
	
	def on_goal_gripper(self, goal_handle_gripper):
		"""Handle a new goal trajectory command."""
		# Checks if the joints are just incorrect
		self.init_trajectory_gripper()
		rospy.loginfo("Updating gripper goal")
		
		if set(goal_handle_gripper.get_goal().trajectory.joint_names) != set(self.gripperprefixedJointNames):
			rospy.logerr("Received a goal with incorrect joint names: (%s)" %
						 ', '.join(goal_handle_gripper.get_goal().trajectory.joint_names))
			goal_handle_gripper.set_rejected()
			return

		if not trajectory_is_finite(goal_handle_gripper.get_goal().trajectory):
			rospy.logerr("Received a goal with infinites or NaNs")
			goal_handle_gripper.set_rejected(text="Received a goal with infinites or NaNs")
			return

		# Checks that the trajectory has velocities
		if not has_velocities(goal_handle_gripper.get_goal().trajectory):
			rospy.logerr("Received a goal without velocities")
			goal_handle_gripper.set_rejected(text="Received a goal without velocities")
			return

		# Inserts the current setpoint at the head of the trajectory
		now = self.robot.getTime()
		point0 = sample_trajectory(self.trajectory_gripper, now - self.trajectory_gripper_t0)
		point0.time_from_start = rospy.Duration(0.0)
		goal_handle_gripper.get_goal().trajectory.points.insert(0, point0)
		self.trajectory_t0 = now

		# Replaces the goal
		self.goal_handle_gripper = goal_handle_gripper
		self.trajectory_gripper = goal_handle_gripper.get_goal().trajectory
		goal_handle_gripper.set_accepted()

	def on_cancel_gripper(self, goal_handle_gripper):
		"""Handle a trajectory cancel command."""
		rospy.loginfo("Canceling gripper trajectory")
		if goal_handle_gripper == self.goal_handle_gripper:
			# stop the motors
			for i in range(len(TrajectoryFollowerGripper.gripperjointNames)):
				self.grippermotors[i].setPosition(self.grippersensors[i].getValue())
			self.goal_handle_gripper.set_canceled()
			self.goal_handle_gripper = None
		else:
			goal_handle_gripper.set_canceled()

	def update_gripper(self):
		# rospy.logwarn("touch_sensor 1" + str(self.TouchSensors[0].getValues()[-1]))
		# rospy.logwarn("touch_sensor 2" + str(self.TouchSensors[1].getValues()[-1]))
		webot_grasp_status = rospy.get_param('/webot_grasp_status')
		

		if self.TouchSensors[0].getValues()[-1] > 100 and self.TouchSensors[1].getValues()[-1] > 100 and webot_grasp_status:
			if self.goal_handle_gripper.get_goal_status().status == 1:
					self.goal_handle_gripper.set_succeeded()

		elif self.robot and self.trajectory_gripper:
			now = self.robot.getTime()
			if (now - self.trajectory_gripper_t0) <= self.trajectory_gripper.points[-1].time_from_start.to_sec():  # Sending intermediate points
				self.last_point_sent_gripper = False
				setpoint_gripper = sample_trajectory(self.trajectory_gripper, now - self.trajectory_gripper_t0)
				
				for i in range(len(self.grippermotors)):
					self.grippermotors[i].setPosition(self.joint_directions[i]*setpoint_gripper.positions[i])
					self.gripper_pos_atual[i] = setpoint_gripper.positions[i]

			elif not self.last_point_sent_gripper:  # All intermediate points sent, sending last point to make sure we reach the goal.
				last_point = self.trajectory_gripper.points[-1]
				state = self.jointStatePublisher.last_joint_states
				position_in_tol = within_tolerance(state.position, last_point.positions, self.joint_goal_tolerances_gripper)
				setpoint_gripper = sample_trajectory(self.trajectory_gripper, self.trajectory_gripper.points[-1].time_from_start.to_sec())
				
				for i in range(len(self.grippermotors)):
					self.grippermotors[i].setPosition(self.joint_directions[i]*setpoint_gripper.positions[i])
					self.gripper_pos_atual[i] = setpoint_gripper.positions[i]

				if self.goal_handle_gripper.get_goal_status().status == 1:
					self.goal_handle_gripper.set_succeeded()