#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point

import numpy as np
import time
from autolab_core import RigidTransform
from frankapy import FrankaArm

fa = FrankaArm(with_gripper=False)
MAX_TIME = 2
DURATION_FACTOR = 2.5
ORIENTATION_Tee = np.array([[0.999989403, 6.35691360e-04, 1.23997835e-03],
                [6.37934120e-04, -9.99988533e-01, -1.80916991e-03],
                [1.23881406e-03, 1.80994177e-03, -9.99997595e-01]])

# TODO: Add collision box on end effector
# TODO: Add internal collision boundaries for arm (DON'T HIT CAMERA, DON'T SET OFF WALLS)
# TODO: Add filtering for min dist between points
# TODO: Reset position is within ground plane
# TODO: Translation to the EE Sword: 2cm down , 10cm out

def strike_callback(data):
    msg_str = "Recieved Location: "+str(data.x)+", "+str(data.y)+", "+str(data.z)
    rospy.loginfo(rospy.get_caller_id() + msg_str)
    des_pose = RigidTransform(rotation = ORIENTATION_Tee,
                            translation = np.array([data.x, data.y, data.z]),
                            from_frame="franka_tool",
                            to_frame="world")
	
    delta = np.linalg.norm(fa.get_pose().translation - des_pose.translation)
    duration = round(delta*DURATION_FACTOR, 1)
    duration = min(MAX_TIME, float(duration))
    fa.goto_pose(des_pose, use_impedance=False, duration=float(duration))
    time.sleep(0.5)
    
def listener():

    fa.reset_joints()	
    # rospy.init_node('frankeArm_striker', anonymous=True)
    rospy.Subscriber("landing_pos", Point, strike_callback, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    listener()