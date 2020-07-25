#! /usr/bin/env python3
import unittest
import rospy
import rostest

from spawn_model_test import SpawnModelTest
from joint_state_topic_test import JointStateTopicTest
from twist_topic_test import TwistTopicTest


class AudibotTests(unittest.TestSuite):
    def __init__(self):
        rospy.init_node('audibot_tests')
        super(AudibotTests, self).__init__()

        tests = [
            SpawnModelTest(False, True, '', 'No TF without prefix'),
            TwistTopicTest('', True),

            SpawnModelTest(False, True, 'test_model', 'No TF with prefix'),
            JointStateTopicTest('test_model'),
            TwistTopicTest('test_model', True),

            SpawnModelTest(True, False, '', 'TF without prefix'),
            SpawnModelTest(True, True, 'test_model', 'TF with prefix'),
        ]
        self.addTests(tests)


if __name__ == '__main__':
    rostest.rosrun('audibot_gazebo', 'audibot_tests', 'audibot_tests.AudibotTests')
