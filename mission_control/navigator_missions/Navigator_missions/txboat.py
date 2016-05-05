#!/usr/bin/env python

from __future__ import division
from twisted.internet import defer
from txros import action, util, tf
from uf_common.msg import MoveToAction, PoseTwistStamped
from uf_common import orientation_helpers
from tf import transformations
from nav_msgs.msg import Odometry
from geometry_msgs.msg import WrenchStamped

class _PoseProxy(object):
    def __init__(self, boat, pose, print_only=False):
        self.boat = boat
        self._pose = pose
        self.print_only = print_only

    def __getattr__(self, name):
        def boat_attr_proxy(*args, **kwargs):
            return _PoseProxy(self.boat, getattr(self._pose, name)(*args, **kwargs), print_only=self.print_only)
        return boat_attr_proxy

    def go(self, *args, **kwargs):
        if self.print_only:
            print 'P: {}, Angles: {}'.format(self._pose.position, transformations.euler_from_quaternion(self._pose.orientation))
            return self.boat._node_handle.sleep(0.1)

        goal = self.boat._moveto_action_client.send_goal(self._pose.as_MoveToGoal(*args, **kwargs))
        return goal.get_result()

    def go_trajectory(self, *args, **kwargs):
        traj = self.boat._trajectory_pub.publish(
            self._pose.as_PoseTwistStamped(*args, **kwargs)
        )
        return traj


class boat(object):
    def __init__(self, node_handle):
        self._node_handle = node_handle

    @util.cancellableInlineCallbacks
    def _init(self):
        self._trajectory_boat = yield self._node_handle.subscribe('trajectory', PoseTwistStamped)
        self._moveto_action_client = yield action.ActionClient(self._node_handle, 'moveto', MoveToAction)
        self._odom_sub = yield self._node_handle.subscribe('odom', Odometry)
        self._wrenchboat = yield self._node_handle.subscribe('wrench', WrenchStamped)
        #The absodom topic has the lat long information ebeded in the nav_msgs/Odometry. the lat long is under pose.pose.position and this is what is reported back in the JSON messages for a chalenge.
        self._absodom_sub = yield self._node_handle.subscribe('absodom', Odometry)
        self._odom_pub = yield self._node_handle.advertise('odom', Odometry)
        defer.returnValue(self)

    @property
    def pose(self):
        last_odom_msg = self._odom_sub.get_last_message()
        pose = orientation_helpers.PoseEditor.from_Odometry(last_odom_msg)
        return pose

    @property
    def odom(self):
        return orientation_helpers.PoseEditor.from_Odometry(
            self._odom_sub.get_last_message())

    @util.cancellableInlineCallbacks
    def last_pose(self):
        last_pose = yield self._odom_sub.get_next_message()
        defer.returnValue(last_pose)

    @property
    def move(self):
        return _PoseProxy(self, self.pose)

    @util.cancellableInlineCallbacks
    def station_hold(self):
        move = yield self.move.set_position(self.odom.position).go()
        defer.returnValue(move)

_boats = {}

@util.cancellableInlineCallbacks
def get_boat(node_handle, need_trajectory=True):
    if node_handle not in _boats:
        _boats[node_handle] = None  # placeholder to prevent this from happening reentrantly
        _boats[node_handle] = yield boat(node_handle)._init()
        # XXX remove on nodehandle shutdown
    defer.returnValue(_boats[node_handle])

