#! /usr/bin/env python
import unittest
import rospy
import rostest

from spawn_model_test import SpawnModelTest


class AudibotTests(unittest.TestSuite):
    def __init__(self):
        rospy.init_node('audibot_tests')
        super(AudibotTests, self).__init__()

        tests = [
            SpawnModelTest(False, False, 'test_model', 'No TF with prefix'),
            SpawnModelTest(False, False, '', 'No TF without prefix'),
            SpawnModelTest(True, False, 'test_model', 'TF with prefix'),
            SpawnModelTest(True, False, '', 'TF without prefix')
        ]
        self.addTests(tests)


if __name__ == '__main__':
    rostest.rosrun('audibot_gazebo', 'audibot_tests', 'audibot_tests.AudibotTests')
