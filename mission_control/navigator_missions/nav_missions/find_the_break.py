#!/usr/bin/env python
import txros
import numpy as np
import sys
import navigator_tools as nt
import sys
import itertools
from navigator_tools import fprint, DBHelper
from navigator_testing_suite import FindTheBreakTestPerception
from navigator_find_the_break import FindTheBreakPerception


class FindTheBreak(object):

    def __init__(self):
        self.perception_defer = None

    def _end_move(self):
        if not self.perception_defer.called:
            self.perception_defer.addErrback(lambda x: x)
            self.perception_defer.cancel()

    @txros.util.cancellableInlineCallbacks
    def do_mission(self, navigator, **kwargs):
        # mission params
        meters_between_buoys = 20
        radius = 40

        # var initialization
        nh = navigator.nh
        test = False
        if "test" in kwargs:
            test = kwargs["test"]
        if test:
            perception = FindTheBreakTestPerception(nh)
        else:
            perception = yield FindTheBreakPerception(nh).init_()
        dbhelper = yield DBHelper(nh).init_(navigator)

        fprint("STARTING FIND THE BREAK", msg_color="green")
        # Get all objects within a certain radius of type unknown or buoy
        nav_pos = yield navigator.tx_pose
        nav_pos = nav_pos[0]
        objects = yield dbhelper.get_objects_in_radius(nav_pos, radius, ["buoy", "unknown"])
        if len(objects) < 2:
            fprint("No objects in proximity", msg_color="red")
            raise Exception("No objects in proximity")

        # Get all combinations and find the ones with the correct distance
        objects_comb = itertools.combinations(objects, 2)
        min_dist = sys.maxint
        min_objs = None
        for o in objects_comb:
            dist = np.linalg.norm(nt.rosmsg_to_numpy(o[0].position) - nt.rosmsg_to_numpy(o[1].position))
            if abs(dist - meters_between_buoys) < min_dist:
                min_dist = dist
                min_objs = o

        fprint("Two objects appear to be the buoys you want, the difference is {} and the names are {}, {}".format(
            min_dist, min_objs[0].name, min_objs[1].name), msg_color="blue")

        # Travel to one, look at the other
        p1 = nt.rosmsg_to_numpy(min_objs[0].position)
        p2 = nt.rosmsg_to_numpy(min_objs[1].position)

        yield navigator.move.set_position(p1).go()
        yield navigator.move.look_at(p2).go()

        # Travel to other slowlys aysynchronously
        move = navigator.move.set_position(p2).go(speed_factor=.5, initial_plan_time=5, move_type='skid')
        move.addErrback(lambda x: x)
        self.perception_defer = perception.count_pipes()
        move.addCallback(self._end_move)

        # Run perception return count, publish it
        num = yield self.perception_defer
        fprint("NUMBER OF PIPES FOUND; {}".format(num), msg_color="green")
        if num > 0 and num < 6:
            yield navigator.mission_params["find_the_break_markers"].set(num)
        else:
            yield navigator.mission_params["find_the_break_markers"].set(3)

        if not move.called:
            move.cancel()

        fprint("COMPLETED FIND THE BREAK", msg_color="green")


@txros.util.cancellableInlineCallbacks
def main(navigator, **kwargs):
    ftb = FindTheBreak()
    yield ftb.do_mission(navigator, **kwargs)


@txros.util.cancellableInlineCallbacks
def safe_exit(navigator, err):
    yield navigator.mission_params["find_the_break_markers"].set(3)
