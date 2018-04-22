#!/usr/bin/env python
from txros import util
from twisted.internet import defer
from navigator import Navigator
import numpy as np
from mil_tools import rosmsg_to_numpy
from geometry_msgs.msg import Vector3Stamped
from mil_tasks_core import TaskException
from scipy.stats import mode


class PingerAndy(Navigator):
    '''
    Mission to run sonar start gate challenge using Andy's sonar system, which produces a vector pointing towards the
    '''
    MAX_TOTEM_DISTANCE = 30.0  # Maximum distance one of the totems surrounding the gates can be
    SAMPLES = 10               # Number of headings to use in selecting gate
    MIN_SAMPLES = 1            # Minimium # of headings to get within timout to attempt mission
    HEADINGS_TIMEOUT = 45      # Timeout in seconds to stop collecting headings even if SAMPLES count is not reached
    TF_TIMEOUT = 1.0           # Max time to wait to transform hydrophones frame to enu
    BEFORE_DISTANCE = 3.0     # Distance infront of gate for first move, meters
    AFTER_DISTANCE = 6.0      # Distance passed gate for second move, meters

    @classmethod
    def init(cls):
        cls.pinger_heading = cls.nh.subscribe("/hydrophones/ping_direction", Vector3Stamped)

    @staticmethod
    def line(p1, p2):
        '''
        Return equation of a line given two 2D points
        https://stackoverflow.com/questions/20677795/how-do-i-compute-the-intersection-point-of-two-lines-in-python
        '''
        A = (p1[1] - p2[1])
        B = (p2[0] - p1[0])
        C = (p1[0] * p2[1] - p2[0] * p1[1])
        return A, B, -C

    @staticmethod
    def intersection(L1, L2):
        '''
        Return point intersection (if it exsists) of two lines given their equations obtained from the line method
        https://stackoverflow.com/questions/20677795/how-do-i-compute-the-intersection-point-of-two-lines-in-python
        '''
        D = L1[0] * L2[1] - L1[1] * L2[0]
        Dx = L1[2] * L2[1] - L1[1] * L2[2]
        Dy = L1[0] * L2[2] - L1[2] * L2[0]
        if D != 0:
            x = Dx / D
            y = Dy / D
            return x, y
        else:
            return None

    @util.cancellableInlineCallbacks
    def get_gates(self):
        totems = []
        for i in range(4):
            query = 'pinger_totem{}'.format(i + 1)
            res = yield self.database_query(query)
            if not res.found:
                raise TaskException(query + ' not found in object database')
            point = rosmsg_to_numpy(res.objects[0].pose.position)[:2]
            totems.append(np.array(point))

        # Create list of gates halfway between each pair of totems
        gates = []
        for i in range(3):
            gates.append((totems[i] + totems[i + 1]) / 2.0)
        defer.returnValue(gates)

    @util.cancellableInlineCallbacks
    def get_pinger_gate(self, gates):
        answers = []
        gates_line = self.line(gates[0], gates[-1])
        i = 0
        while i < self.SAMPLES:
            heading = yield self.pinger_heading.get_next_message()
            self.frame = heading.header.frame_id
            self.send_feedback('Heading sample {}/{} recieved'.format(i + 1, self.SAMPLES))
            stamp = None #heading.header.stamp if not heading.header.stamp.is_zero() else None
            hydrophones_to_enu = yield util.wrap_timeout(self.tf_listener.get_transform('enu', heading.header.frame_id, time=stamp), self.TF_TIMEOUT).addErrback(lambda _: None)
            if hydrophones_to_enu is None:
                self.send_feedback('Transform failed, continuing...')
                continue
            hydrophones_origin = hydrophones_to_enu._p[0:2]
            heading = rosmsg_to_numpy(heading.vector)
            heading_enu = hydrophones_to_enu.transform_vector(heading)
            heading_enu = heading_enu[0:2] / np.linalg.norm(heading_enu[0:2])
            pinger_line = self.line(hydrophones_origin, hydrophones_origin + heading_enu)

            # Find intersection of these two lines. This is the approximate position of the pinger
            intersection = self.intersection(pinger_line, gates_line)
            if intersection is None:
                self.send_feedback('Sample is insane! Ignoring.')
                continue
            distances = []
            for gate in gates:
                distances.append(np.linalg.norm(gate[0:2] - intersection))
            argmin = np.argmin(np.array(distances))
            answers.append(argmin)
            i += 1
            self.send_feedback('Current heading points to gate {}'.format(argmin + 1))
        if len(answers) < self.MIN_SAMPLES:
            raise TaskException('not enough headings', {'headings': len(answers), 'timeout': self.HEADINGS_TIMEOUT})
        answers = np.array(answers)  # Everything's better as a numpy array
        gate_mode = mode(answers)[0][0]
        self.send_feedback('Best gate is gate {} based on mode of {} samples'.format(gate_mode + 1, len(answers)))
        defer.returnValue(gate_mode)

    @util.cancellableInlineCallbacks
    def run(self, args):
        # Get position of 3 gates based on position of totems
        gates = yield self.get_gates()
        self.send_feedback('Getting hydrophone samples')
        pinger_gate_index = yield self.get_pinger_gate(gates)
        pinger_gate = gates[pinger_gate_index]

        between_vector = gates[0] - gates[-1]
        # Rotate that vector to point through the buoys
        c = np.cos(np.radians(90))
        s = np.sin(np.radians(90))
        R = np.array([[c, -s], [s, c]])
        direction_vector = R.dot(between_vector)
        direction_vector /= np.linalg.norm(direction_vector)
        position = self.pose[0][:2]
        if np.linalg.norm(position - (pinger_gate + direction_vector)) > np.linalg.norm(position - (pinger_gate - direction_vector)):
            direction_vector = -direction_vector

        before = np.append(pinger_gate + direction_vector * self.BEFORE_DISTANCE, 0)
        after = np.append(pinger_gate - direction_vector * self.AFTER_DISTANCE, 0)

        self.send_feedback('Moving in front of gate')
        yield self.move.set_position(before).look_at(after).go()
        self.send_feedback('Going through')
        yield self.move.set_position(after).go()

        defer.returnValue('My god it actually worked!')
