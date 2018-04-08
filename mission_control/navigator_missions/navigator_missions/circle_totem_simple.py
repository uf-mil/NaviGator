#!/usr/bin/env python
import txros
from twisted.internet import defer
import mil_tools
import numpy as np
from navigator import Navigator


class CircleTotemSimple(Navigator):
    CIRCLE_DISTANCE = 6.0
    SPEED_FACTOR = 0.5
    DIRECTIONS = {'RED': 'cw', 'GREEN': 'ccw', 'BLUE': 'cw', 'YELLOW': 'ccw'}

    @txros.util.cancellableInlineCallbacks
    def run(self, parameters):
        # Get colors of intrest and directions
        '''
        c1 = self.mission_params['scan_the_code_color1'].get()
        c2 = self.mission_params['scan_the_code_color2'].get()
        c3 = self.mission_params['scan_the_code_color3'].get()
        colors = [c1, c2, c3]
        '''

        colors = ['RED', 'GREEN', 'BLUE']
        # '''
        self.totems = yield self.database_query("totem", raise_exception=False)
        if not self.totems.found:
            defer.returnValue('No totems found')
        targets = []
        for color in colors:
            totem = self.get_colored_totem(color.lower())
            if totem is None:
                self.send_feedback('Totem {} not found'.format(totem))
                yield self.nh.sleep(0.1)
            else:
                targets.append((mil_tools.rosmsg_to_numpy(totem.pose.position), color))
        # '''
        '''
        # Tmp for testing
        targets = [([5, 0, 0], 'RED'),
                   ([0, -5, 0], 'GREEN'),
                   ([0, 5, 0], 'BLUE')]
        '''
        if len(targets) == 0:
            defer.returnValue('No totems with correct color found')
        for target in targets:
            color = target[1]
            position = target[0]
            direction = self.DIRECTIONS[color]
            self.send_feedback('Attempting to circle {} {}'.format(color, direction))
            self.send_feedback('Moving in front of totem')
            yield self.move.look_at(position).set_position(position).backward(self.CIRCLE_DISTANCE).go()
            self.send_feedback('Circling!')
            yield self.move.circle_point(position, direction=direction).go()
        defer.returnValue('Success')

    def get_colored_totem(self, color):
        if not self.totems.found:
            return None
        color_matching = [totem for totem in self.totems.objects
                          if totem.labeled_classification == 'totem_{}'.format(color)]
        if len(color_matching) == 0:
            return None
        sort = sorted(color_matching,
                      key=lambda totem: np.linalg.norm(self.pose[0] - mil_tools.rosmsg_to_numpy(totem.pose.position)))
        return sort[0]
