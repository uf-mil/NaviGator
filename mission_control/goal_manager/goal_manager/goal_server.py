#!/usr/bin/env python

import rospy
import actionlib
import goal_manager.msg

""" This server is currently a barebones server and is used only to test the
    client node at the moment
"""


class goal_server(object):

    def __init__(self):
        self.server = actionlib.SimpleActionServer('goal', goal_manager.msg.current_goalAction,
                                                   execute_cb=self.execute_cb, auto_start=False
                                                   )
        self.feedback = goal_manager.msg.current_goalFeedback()
        self.result = goal_manager.msg.current_goalResult()
        self.server.start()
        self.success = False

    def send_feedback(self):

        self.success = True
        # append the seeds for the current_goal sequence
        self.feedback.sequence = []

        # check that preempt has not been requested by the client
        if self.server.is_preempt_requested():
            self.server.set_preempted()
            self.success = False

        self.feedback.sequence.append(0)
        self.server.publish_feedback(self.feedback)

    def execute_cb(self, goal):

        while not rospy.is_shutdown():
            self.send_feedback()
            if self.success:
                break

        self.result.flag = True
        self.server.set_succeeded(self.result)


if __name__ == '__main__':
    rospy.init_node('goal_server')
    goal_server()
    rospy.spin()
