#! /usr/bin/env python3
import unittest
import rospy
import rospkg
from tf import LookupException, ExtrapolationException, TransformListener
import os
import sys
import shlex
import xacro
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose
import io


class SpawnModelTest(unittest.TestCase):
    def __init__(self, pub_tf, leave_spawned, robot_name, test_name):
        super(SpawnModelTest, self).__init__('spawnModelTest')
        self.test_name = '[' + test_name + '] '
        self.robot_name = robot_name
        if len(robot_name) == 0:
            self.model_name = 'audibot'
        else:
            self.model_name = robot_name
        self.model_xml = ''
        self.pub_tf = pub_tf
        self.leave_spawned = leave_spawned
        self.spawn_srv = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        self.tf_listener = TransformListener()

    def setUp(self):
        self.model_xml = self.__parseXacro(self.pub_tf, self.robot_name)

    def tearDown(self):
        if not self.leave_spawned:
            delete_srv = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
            try:
                delete_srv.wait_for_service(1.0)
                delete_srv(model_name=self.model_name)
            except rospy.ServiceException:  # service call failed
                pass
            except rospy.ROSInterruptException:  # ROS shutdown during timeout
                pass
            except rospy.ROSException:  # timeout expired
                pass

    def spawnModelTest(self):
        results = self.__callSpawnService(self.model_xml)
        self.assertTrue(results[0], self.test_name + results[1])

        if self.pub_tf:
            self.assertTrue(self.__tfLookupWithTimeout(self.robot_name + '/base_footprint', 'world'),
                            msg=self.test_name + 'TF lookup failed')
        else:
            self.assertFalse(self.__tfLookupWithTimeout(self.robot_name + '/base_footprint', 'world'),
                             msg=self.test_name + 'lookupTransform did not fail')

    def __tfLookupWithTimeout(self, child_frame, parent_frame):
        success = False
        timeout_t = rospy.Time.now() + rospy.Duration(5)
        while not rospy.is_shutdown() and (timeout_t - rospy.Time.now()).to_sec() > 0:
            try:
                self.tf_listener.lookupTransform(child_frame, parent_frame, rospy.Time(0))
                success = True
                break
            except LookupException:
                pass

            rospy.sleep(0.01)
        return success

    def __callSpawnService(self, model_xml):

        pose = Pose()
        pose.orientation.w = 1

        try:
            self.spawn_srv.wait_for_service(30.0)
            resp = self.spawn_srv(model_name=self.model_name,
                                  model_xml=model_xml,
                                  robot_namespace=self.robot_name,
                                  reference_frame='world',
                                  initial_pose=pose
                                  )
            if resp.success:
                return True, ''
            else:
                return False, 'Spawn service call failed'
        except rospy.ServiceException:
            return False, 'Service call failed'
        except rospy.ROSInterruptException:
            return False, 'ROS interrupt before spawn service call'
        except rospy.ROSException:  # timeout in wait_for_service()
            return False, 'Spawn model service unavailable'

    @staticmethod
    def __parseXacro(pub_tf, robot_name):
        urdf_path = rospkg.RosPack().get_path('audibot_description') + '/urdf/audibot.urdf.xacro'

        # Construct xacro argument string
        arg_str = 'placeholder '  # First argv argument technically is the program path

        # --inorder flag only needed in Kinetic
        if os.environ['ROS_DISTRO'] == 'kinetic':
            arg_str += '--inorder '
        arg_str += urdf_path                            # Name of the URDF file to load

        if len(robot_name) > 0:
            arg_str += ' robot_name:=' + robot_name     # Runtime name and TF prefix

        if pub_tf:                                      # Publish a perfectly accurate transform from world-->footprint
            arg_str += ' pub_tf:=true'
        else:
            arg_str += ' pub_tf:=false'
        arg_str += ' blue:=false'

        # Make arg_str look like an argv array and set actual argv to it
        sys.argv = shlex.split(arg_str)

        # Redirect stdout from xacro to a StringIO object
        old_std_out = sys.stdout
        sys.stdout = xml_output = io.StringIO()
        xacro.main()
        sys.stdout = old_std_out
        return xml_output.getvalue()
