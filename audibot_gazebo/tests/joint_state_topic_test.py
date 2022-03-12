#! /usr/bin/env python3
import unittest
import rospy
import collections
from sensor_msgs.msg import JointState


class JointStateTopicTest(unittest.TestCase):
    def __init__(self, robot_name):
        super(JointStateTopicTest, self).__init__('jointStateTopicTest')
        self.robot_name = robot_name
        self.joint_states = JointState()
        self.joint_state_topic = self.robot_name + '/joint_states'

    def setUp(self):
        self.sub_joints = rospy.Subscriber(self.joint_state_topic, JointState, self.__recvJointStates)

    def tearDown(self):
        self.sub_joints.unregister()

    def jointStateTopicTest(self):

        # Wait for a JointState message sample on the appropriate topic
        timeout_t = rospy.Time.now() + rospy.Duration(5)
        while not rospy.is_shutdown() and (timeout_t - rospy.Time.now()).to_sec() > 0:
            if self.joint_states.header.stamp != rospy.Time(0):
                break
            rospy.sleep(0.01)

        self.assertTrue(self.joint_states.header.stamp != rospy.Time(0),
                        msg='Joint states topic [%s] not received' % self.joint_state_topic)

        # Make sure the joint names are correct
        correct_joints = ['steer_fl_joint', 'steer_fr_joint', 'wheel_fl_joint', 'wheel_fr_joint', 'wheel_rl_joint', 'wheel_rr_joint']
        self.assertTrue(collections.Counter(self.joint_states.name) == collections.Counter(correct_joints),
                        msg='Joint names %s incorrect, should be %s' % (str(self.joint_states.name), str(correct_joints))
                        )

        # Make sure position array has the appropriate number of elements
        self.assertTrue(len(self.joint_states.position) == len(correct_joints),
                        msg='Joint state position array not correct length (len = %d, should be %d)' % (len(self.joint_states.position), len(correct_joints))
                        )

        # Make sure velocity array has the appropriate number of elements
        self.assertTrue(len(self.joint_states.velocity) == len(correct_joints),
                        msg='Joint state velocity array not correct length (len = %d, should be %d)' % (len(self.joint_states.velocity), len(correct_joints))
                        )

    def __recvJointStates(self, msg):
        self.joint_states = msg
