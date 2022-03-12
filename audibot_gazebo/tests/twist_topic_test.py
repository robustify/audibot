#! /usr/bin/env python3
import unittest
import rospy
from geometry_msgs.msg import TwistStamped
from gazebo_msgs.srv import DeleteModel


class TwistTopicTest(unittest.TestCase):
    def __init__(self, robot_name, delete_model):
        super(TwistTopicTest, self).__init__('twistTopicTest')
        self.robot_name = robot_name
        self.twist = TwistStamped()
        self.twist_topic = self.robot_name + '/twist'
        self.delete_model = delete_model

    def setUp(self):
        self.sub_twist = rospy.Subscriber(self.twist_topic, TwistStamped, self.__recvTwist)

    def tearDown(self):
        self.sub_twist.unregister()

        if self.delete_model:
            if len(self.robot_name) == 0:
                model_name = 'audibot'
            else:
                model_name = self.robot_name
            delete_srv = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

            try:
                delete_srv.wait_for_service(1.0)
                delete_srv(model_name=model_name)
            except rospy.ServiceException:  # service call failed
                pass
            except rospy.ROSInterruptException:  # ROS shutdown during timeout
                pass
            except rospy.ROSException:  # timeout expired
                pass

    def twistTopicTest(self):

        # Wait for a twist feedback message sample on the appropriate topic
        timeout_t = rospy.Time.now() + rospy.Duration(1)
        while not rospy.is_shutdown() and (timeout_t - rospy.Time.now()).to_sec() > 0:
            if self.twist.header.stamp != rospy.Time(0):
                break
            rospy.sleep(0.01)

        self.assertTrue(self.twist.header.stamp != rospy.Time(0),
                        msg='TwistStamped topic [%s] not received' % self.twist_topic)

        # Make sure frame_id is correct
        if len(self.robot_name) > 0:
            correct_frame_id = self.robot_name + '/base_footprint'
        else:
            correct_frame_id = 'base_footprint'

        self.assertEqual(first=self.twist.header.frame_id,
                         second=correct_frame_id,
                         msg='TwistStamped frame_id [%s] should be [%s]' % (self.twist.header.frame_id, correct_frame_id)
                         )

    def __recvTwist(self, msg):
        self.twist = msg
