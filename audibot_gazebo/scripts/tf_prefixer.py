#! /usr/bin/env python3
import rospy
import rosparam
from tf2_msgs.msg import TFMessage

rospy.init_node('tf_prefixer')

pub_tf = rospy.Publisher("/tf", TFMessage, queue_size=10)
pub_tf_static = rospy.Publisher("/tf_static", TFMessage, queue_size=10)
prefix = rospy.get_param('~tf_prefix')

def tf_callback(data):
    global pub_tf, prefix
    n_transforms = len(data.transforms)
    for i in range(n_transforms):
        data.transforms[i].header.frame_id = prefix + '/' + data.transforms[i].header.frame_id
        data.transforms[i].child_frame_id = prefix + '/' + data.transforms[i].child_frame_id
    pub_tf.publish(data)

def tf_static_callback(data):
    global pub_tf_static, prefix
    n_transforms = len(data.transforms)
    for i in range(n_transforms):
        data.transforms[i].header.frame_id = prefix + '/' + data.transforms[i].header.frame_id
        data.transforms[i].child_frame_id = prefix + '/' + data.transforms[i].child_frame_id
    pub_tf_static.publish(data)

tf_listener = rospy.Subscriber("tf", TFMessage, tf_callback)
tf_static_listener = rospy.Subscriber("tf_static", TFMessage, tf_static_callback)

rospy.spin()
