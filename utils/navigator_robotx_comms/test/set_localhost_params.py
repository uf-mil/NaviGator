#!/usr/bin/env python

import rospy

if __name__ == "__main__":

    rospy.set_param('td_ip', '127.0.0.1')
    rospy.set_param('td_port', 1337)