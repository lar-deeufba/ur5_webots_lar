"""Joint state publisher."""

import rospy
from sensor_msgs.msg import JointState


class JointStatePublisher(object):
    """Publish as a ROS topic the joint state."""
    jointNames = [
        'shoulder_pan_joint',
        'shoulder_lift_joint',
        'elbow_joint',
        'wrist_1_joint',
        'wrist_2_joint',
        'wrist_3_joint'
    ]
    def __init__(self, robot, jointPrefix, nodeName):
        """Initialize the motors, position sensors and the topic."""
        self.robot = robot
        self.jointPrefix = jointPrefix
        self.motors = []
        self.sensors = []
        self.timestep = int(robot.getBasicTimeStep())
        self.last_joint_states = None
        self.previousTime = 0
        self.previousPosition = []
        for name in JointStatePublisher.jointNames:
            self.motors.append(robot.getDevice(name))
            self.sensors.append(robot.getDevice(name + '_sensor'))
            self.sensors[-1].enable(self.timestep)
            self.previousPosition.append(0)
        self.publisher = rospy.Publisher(nodeName + 'joint_states', JointState, queue_size=1)

    def publish(self):
        """Publish the 'joint_states' topic with up to date value."""
        msg = JointState()
        msg.header.stamp = rospy.get_rostime()
        msg.header.frame_id = "From simulation state data"
        msg.name = [s + self.jointPrefix for s in JointStatePublisher.jointNames]
        msg.position = []
        timeDifference = self.robot.getTime() - self.previousTime
        for i in range(len(self.sensors)):
            value = self.sensors[i].getValue()
            msg.position.append(value)
            msg.velocity.append((value - self.previousPosition[i]) / timeDifference if timeDifference > 0 else 0.0)
            self.previousPosition[i] = value
        msg.effort = [0] * 7
        self.publisher.publish(msg)
        self.last_joint_states = msg
        self.previousTime = self.robot.getTime()
