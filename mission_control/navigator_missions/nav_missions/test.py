#!/usr/bin/env python
import txros
import tf
import numpy as np
import navigator_tools
import math as math
import matplotlib.pyplot as plt
import rospy
import navigator_msgs.srv as navigator_srvs
@txros.util.cancellableInlineCallbacks

def main(navigator):
    navigator.change_wrench("autonomous")
    circle = navigator.move.circle_point([3,17,0], 5)
    for p in circle:
      yield p.go()
