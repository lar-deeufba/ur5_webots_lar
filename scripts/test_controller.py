#!/usr/bin/python
import actionlib
import numpy as np

# ROS Msgs
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, JointTolerance
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler

# ROS
import rospy
from tf import TransformListener

# IK TRAJ
from trac_ik_python.trac_ik import IK
ur5_joint_names = rospy.get_param("/ur5_joint_names")

class ur5_test_controller(object):
	def __init__(self, joint_values = None):
		rospy.init_node('test_controller', log_level=rospy.FATAL)
		self.joint_values_home = joint_values

		self.tf = TransformListener()
		rospy.Subscriber('/joint_states', JointState, self.ur5_actual_position_callback, queue_size=1)

		self.client = actionlib.SimpleActionClient('/follow_joint_trajectory', FollowJointTrajectoryAction)
		print("Waiting for server (arm_controller)...")
		self.client.wait_for_server()
		print("Connected to server (arm_controller)")

		self.robotiq_joint_name = rospy.get_param("/robotiq_joint_name")

		# Action clients
		self.client_gripper = actionlib.SimpleActionClient('gripper_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
		print("Waiting for server (gripper_controller)...")
		self.client_gripper.wait_for_server()
		print("Connected to server (gripper_controller)")

	def ur5_actual_position_callback(self, joint_values_from_ur5):
		"""Get UR5 joint angles"""
		self.th1, self.th2, self.th3, self.th4, self.th5, self.th6 = joint_values_from_ur5.position
		self.actual_position = [self.th1, self.th2, self.th3, self.th4, self.th5, self.th6]
	
	def all_close(self, goal, tolerance=0.00005):
		"""Wait until goal is reached in configuration space"""
		error = np.sum([(self.actual_position[i] - goal[i])**2 for i in range(6)])
		while not rospy.is_shutdown() and error > tolerance:
			error = np.sum([(self.actual_position[i] - goal[i])**2 for i in range(6)])
	
	def get_ik(self, position):
		"""
		Get the inverse kinematics

		Please refer to the trac_ik_python for more info about its usage
		"""
		ik_solver = IK("base_link", "wrist_3_link", solve_type="Distance")
		ik_solver.set_joint_limits([-6.2831854820251465, -2.3, -2.7, -3.14, -6.2831854820251465, -6.2831854820251465],
									[ 6.2831854820251465, -0.5, -0.7,  3.14,  6.2831854820251465,  6.2831854820251465])

		reference_pose = [0.2201039360819781, -1.573845095552878, -1.521853400505349, -1.6151347051274518, 1.5704492904506875, 0.0]
		
		q = quaternion_from_euler(-1.57, 0.0, 1.57)
		tentativas = 0
		max_tentativas = 50
		sol = None
		while tentativas < max_tentativas and not rospy.is_shutdown() and sol is None:
			# In webots the x corresponds to y 
			sol = ik_solver.get_ik(reference_pose, 
								position[1], position[0], position[2], q[0], q[1], q[2], q[3])
			tentativas = tentativas + 1
		
		if sol is not None:
			sol = list(sol)
			return sol
		else:
			return None
	
	def __build_goal_message_ur5(self):
		goal = FollowJointTrajectoryGoal()
		goal.trajectory = JointTrajectory()
		goal.trajectory.joint_names = ur5_joint_names
		goal.goal_tolerance.append(JointTolerance('joint_tolerance', 0.1, 0.1, 0))
		goal.goal_time_tolerance = rospy.Duration(5,0)
		return goal

	def quintic_trajectory_planner(self, grasp_position, grasp_step, way_points_number, movement):
		"""Quintic Trajectory Planner"""
		if grasp_step == 'move':
			joint_pos = self.get_ik(grasp_position)
		
		if joint_pos is not None:
			if movement=='slow':
				final_traj_duration = 500.0 # total iteractions
			elif movement=='fast':
				final_traj_duration = 300.0
			v0 = a0 = vf = af = 0
			t0 = 5.0
			tf = (t0 + final_traj_duration) / way_points_number # tf by way point
			t = tf / 10 # for each movement
			ta = tf / 10 # to complete each movement
			a = [0.0]*6
			pos_points, vel_points, acc_points = [0.0]*6, [0.0]*6, [0.0]*6
			
			goal = self.__build_goal_message_ur5()
			for i in range(6):
				q0 = self.actual_position[i]
				qf = joint_pos[i]
				b = np.array([q0,v0,a0,qf,vf,af]).transpose()
				m = np.array([[1, t0, t0**2,   t0**3,    t0**4,    t0**5],
							[0,  1,  2*t0, 3*t0**2,  4*t0**3,  5*t0**4],
							[0,  0,     2,    6*t0, 12*t0**2, 20*t0**3],
							[1, tf, tf**2,   tf**3,    tf**4,    tf**5],
							[0,  1,  2*tf, 3*tf**2,  4*tf**3,  5*tf**4],
							[0,  0,     2,    6*tf, 12*tf**2, 20*tf**3]])
				a[i] = np.linalg.inv(m).dot(b)
			for i in range(way_points_number):
				for j in range(6):
					pos_points[j] =   a[j][0] +   a[j][1]*t +    a[j][2]*t**2 +    a[j][3]*t**3 +   a[j][4]*t**4 + a[j][5]*t**5
					vel_points[j] =   a[j][1] + 2*a[j][2]*t +  3*a[j][3]*t**2 +  4*a[j][4]*t**3 + 5*a[j][5]*t**4
					acc_points[j] = 2*a[j][2] + 6*a[j][3]*t + 12*a[j][4]*t**2 + 20*a[j][5]*t**3
				goal.trajectory.points.append(JointTrajectoryPoint(positions=pos_points,
																velocities=vel_points,
																accelerations=acc_points,
																time_from_start=rospy.Duration(t))) #default 0.1*i + 5
				t += ta
			return True, goal, joint_pos
		else:
			print('Could not find a IK solution')
			return False, None, None
	
	def traj_planner_request(self, grasp_position, movement='slow'):
		"""Quintic Trajectory Planner"""
		grasp_step='move'
		way_points_number=10

		status, goal, joint_pos = self.quintic_trajectory_planner(grasp_position, grasp_step, way_points_number, movement)
		if status:
			self.client.send_goal(goal)
			self.all_close(joint_pos)
		else:
			print("Could not retrieve any IK solution")
		
	def gripper_send_position_goal(self, position=0.3, action='close'):
		"""Send position goal to the gripper"""

		duration = 6
		velocity = 0.08
		if action == 'open':
			position = 0.1
			grasp_status = False
		elif action == 'close':
			position = 0.7
			grasp_status = True

		goal = FollowJointTrajectoryGoal()
		goal.trajectory = JointTrajectory()
		goal.trajectory.joint_names = self.robotiq_joint_name
		
		goal.trajectory.points.append(JointTrajectoryPoint(positions=[position]*6,
														   velocities=[velocity]*6,
														   accelerations=[0.0]*6,
														   time_from_start=rospy.Duration(duration)))
		rospy.set_param('/webot_grasp_status', grasp_status)
		self.client_gripper.send_goal(goal)
		self.client_gripper.wait_for_result()
		

def main():
	ur5_controller = ur5_test_controller()
	
	point_init_home = [0.5, 0.1, 0.50]
	joint_values_home = ur5_controller.get_ik(point_init_home)

	ur5_controller.joint_values_home = joint_values_home
	depth_shot_point = [0.5, 0.1, 0.35]

	i = 0
	while not rospy.is_shutdown():
		# Send the robot to the custom HOME position
		try:
			opt = int(raw_input("\n\n==== Escolha uma opcao: \
						\n [1] - Ir para HOME\
						\n [2] - Ir para posicao de GRASP \
						\n [3] - Abrir a garra \
						\n [4] - Fechar a garra \
						\nOpcao: "))
		except:
			print("Escolha uma opcao correta.")
			continue

		if opt == 1:
			ur5_controller.traj_planner_request(point_init_home, movement='fast')
		elif opt == 2:
			ur5_controller.traj_planner_request(depth_shot_point, movement='fast')
		elif opt == 3:
			ur5_controller.gripper_send_position_goal(action='open')
		elif opt == 4:
			ur5_controller.gripper_send_position_goal(action='close')

		rospy.sleep(1.0)
		
if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		print("Program interrupted before completion")