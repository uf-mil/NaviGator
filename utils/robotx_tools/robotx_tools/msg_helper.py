from __future__ import division
import numpy as np
from geometry_msgs.msg import Quaternion

'''
    A file to assist with some messages that are commonly used 
    on the Wam-V
'''

def ros_to_np_3D(msg):

    xyz_array = numpy.array(([msg.x, msg.y. msg.z]))
    return xyz_array
