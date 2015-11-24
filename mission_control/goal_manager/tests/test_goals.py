#!/usr/bin/env python

import unittest
import rospy
from nav_msgs.msg import Odometry
from goal_manager import goal_proxy

class TestROSTools(unittest.TestCase):

    def test_down(self):
        sub = goal_proxy(testing=True)
        to_send = Odometry()
        to_send.pose.pose.position.z -= 10
        test = sub.move_down(10)
        self.assertEqual(test, to_send, "Waypoint down test failed")

    def test_up(self):
        sub = goal_proxy(testing=True)
        to_send = Odometry()
        to_send.pose.pose.position.z += 10
        test = sub.move_up(10)
        self.assertEqual(test, to_send, "Waypoint up test failed")

    def test_left(self):
        sub = goal_proxy(testing=True)
        to_send = Odometry()
        to_send.pose.pose.position.y -= 10
        test = sub.move_left(10)
        self.assertEqual(test, to_send, "Waypoint left test failed")

    def test_right(self):
        sub = goal_proxy(testing=True)
        to_send = Odometry()
        to_send.pose.pose.position.y += 10
        test = sub.move_right(10)
        self.assertEqual(test, to_send, "Waypoint right test failed")

    def test_forward(self):
        sub = goal_proxy(testing=True)
        to_send = Odometry()
        to_send.pose.pose.position.x += 10
        test = sub.move_forward(10)
        self.assertEqual(test, to_send, "Waypoint forward test failed")

    def test_back(self):
        sub = goal_proxy(testing=True)
        to_send = Odometry()
        to_send.pose.pose.position.x -= 10
        test = sub.move_back(10)
        self.assertEqual(test, to_send, "Waypoint back test failed")


if __name__ == '__main__':
    rospy.init_node("goal_client_test", anonymous=True)
    suite = unittest.TestLoader().loadTestsFromTestCase(TestROSTools)
    unittest.TextTestRunner(verbosity=2).run(suite)
