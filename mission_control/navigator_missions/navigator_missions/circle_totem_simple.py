#!/usr/bin/env python
import txros
from twisted.internet import defer
import mil_tools
import numpy as np
from navigator import Navigator


class CircleTotemSimple(Navigator):
    CIRCLE_DISTANCE = 6.0
    SPEED_FACTOR = 0.5
    DIRECTIONS = {'RED': 'cw', 'GREEN': 'ccw', 'BLUE': 'cw', 'YELLOW': 'ccw', 'WHITE': 'ccw'}

    @txros.util.cancellableInlineCallbacks
    def run(self, parameters):
        # Get colors of intrest and directions
        '''
        c1 = self.mission_params['scan_the_code_color1'].get()
        c2 = self.mission_params['scan_the_code_color2'].get()
        c3 = self.mission_params['scan_the_code_color3'].get()
        colors = [c1, c2, c3]
        '''

        colors = ['BLUE', 'WHITE']
        # '''
        targets = []
        for color in colors:
            totem = yield self.get_colored_totem(color.lower())
            if totem is None:
                self.send_feedback('Totem {} not found'.format(color))
            else:
                self.send_feedback("Totem {} found!".format(color))
                targets.append((mil_tools.rosmsg_to_numpy(totem.pose.position), color))
            yield self.nh.sleep(0.1)
        # '''
        '''
        # Tmp for testing
        targets = [([5, 0, 0], 'RED'),
                   ([0, -5, 0], 'GREEN'),
                   ([0, 5, 0], 'BLUE')]
        '''
        if len(targets) == 0:
            defer.returnValue('No totems with correct color found')
        self.send_feedback('Targets: {}'.format(targets))
        yield self.nh.sleep(0.1)
        for i in range(len(targets)):
            color = targets[i][1]
            position = targets[i][0]
            direction = self.DIRECTIONS[color]
            self.send_feedback('Attempting to circle {} {}'.format(color, direction))
            self.send_feedback('Moving in front of totem')
            yield self.nh.sleep(0.1)
            res = yield self.move.look_at(position).set_position(position).backward(self.CIRCLE_DISTANCE).go()
            self.send_feedback('RESDFDFD {}'.format(res))
            self.send_feedback('Circling!')
            yield self.nh.sleep(0.1)
            res = yield self.move.circle_point(position, direction=direction).go()
            self.send_feedback('Got {}'.format(res))
            self.send_feedback('Done circling')
            yield self.nh.sleep(0.1)
        defer.returnValue('Success')

    @txros.util.cancellableInlineCallbacks
    def get_colored_totem(self, color):
        totems = yield self.database_query("totem_{}".format(color), raise_exception=False)
        if not totems.found:
            defer.returnValue(None)
        sort = sorted(totems.objects,
                      key=lambda totem: np.linalg.norm(self.pose[0] - mil_tools.rosmsg_to_numpy(totem.pose.position)))
        defer.returnValue(sort[0])
