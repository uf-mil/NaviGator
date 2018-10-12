#!/usr/bin/env python
from txros import util
from navigator_missions.navigator import Navigator
from geometry_msgs.msg import Vector3Stamped
import numpy as np
from mil_tools import rosmsg_to_numpy
from twisted.internet import defer
import math


class StartGateJaxon(Navigator):
    @classmethod
    def init(cls):
        cls.pinger_heading = cls.nh.subscribe("/hydrophones/ping_direction", Vector3Stamped)

    @util.cancellableInlineCallbacks
    def run(self, args):
        gate_results = yield self.find_gates()
        gate_centers = gate_results[0]
        gates_line = gate_results[1]
        # print('gates line ' + str(gates_line))
        pinger_line = yield self.get_pinger_line()
        # print('pinger line ' + str(pinger_line))
        pinger_gate_index = self.determine_pinger_gate(gate_centers, gates_line, pinger_line)
        pinger_gate = gate_centers[pinger_gate_index]

        # print('gates centers ' + str(gate_centers))
        # print('gate center center ' + str(gate_centers[1]))

        traversal_points = yield self.get_traversal_points(gates_line, pinger_gate, gate_centers[1])

        # print(pinger_gate)
        # print(traversal_points)
        self.send_feedback(' Gate identified as ' + str(pinger_gate_index + 1))
        # self.send_feedback(' Navigating to ' + str(traversal_points))
        yield self.move.set_position(traversal_points[0]).look_at(traversal_points[1]).go()
        yield self.move.set_position(traversal_points[1]).look_at(traversal_points[2]).go()
        yield self.move.set_position(traversal_points[2]).go()

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

    @staticmethod
    def perpendicular(L):
        v1 = [L[1], -L[0], 0]
        r = v1[0] ** 2 + v1[1] ** 2
        v1 = [v1[0] / r, v1[1] / r]
        # print('v1=' + str(v1))
        v2 = [0, 0, 1]
        pvec = np.cross(v1, v2)
        r = math.sqrt(pvec[0] ** 2 + pvec[1] ** 2)
        # print('perp vec non normal ' + str(pvec))
        # print('perp vec r ' + str(r))
        # print('perp vec normal ' + str([pvec[0]/r, pvec[1]/r, 0]))
        return [pvec[0] / r, pvec[1] / r, 0]

    @util.cancellableInlineCallbacks
    def find_nearest_objects(self, name, number):
        boat_enu = yield self.tx_pose
        boat_enu = boat_enu[0]
        objects = yield self.database_query(name)
        print(objects)
        assert objects.found, name + " not found"
        assert len(objects.objects) >= number, "Not enough " + name + " found. Need " + str(number) + " found " + str(
            len(objects.objects))
        # print('objects len' + str(len(objects.objects)))

        totems_dists = []

        for obj in objects.objects:
            obj_pos = rosmsg_to_numpy(obj.pose.position)[:2]
            obj_dst = (boat_enu[0] - obj_pos[0]) ** 2 + (boat_enu[1] - obj_pos[1]) ** 2
            # totems_dists = np.append(totems_dists, (obj_pos, obj_dst))
            totems_dists.append((obj_pos, obj_dst))

        # print('totem dist count ' + str(len(totems_dists)))
        # print('totem dists ' + str(totems_dists))
        totems_dists = sorted(totems_dists, key=lambda td: td[1])
        nearest_objects = list(map(lambda td: td[0], totems_dists))[:number]
        defer.returnValue(nearest_objects)

    @util.cancellableInlineCallbacks
    def find_gates(self):
        t1 = yield self.find_nearest_objects("totem_red", 1)
        # print (t1)
        t1 = t1[0]
        # t1 = (55 + 15, -50 + 0)
        white_totems = yield self.find_nearest_objects("totem_white", 2)
        t2 = white_totems[0]
        # t2 = (55 + 5, -50 + 0)
        t3 = white_totems[1]
        # t3 = (55 + -5, -50 + -0)
        t4 = yield self.find_nearest_objects("totem_green", 1)
        t4 = t4[0]
        # t4 = (55 + -15, -50 + 0)

        # print('totem 1: ' + str(t1))
        # print('totem 2: ' + str(t2))
        # print('totem 3: ' + str(t3))
        # print('totem 4: ' + str(t4))

        if (t2[0] - t1[0]) ** 2 + (t2[1] - t1[1]) ** 2 < (t3[0] - t1[0]) ** 2 + (t3[1] - t1[1]) ** 2:
            gate_totems = [t1, t2, t3, t4]
        else:
            gate_totems = [t1, t3, t2, t4]

        gate_centers = [((gate_totems[0][0] + gate_totems[1][0]) / 2, (gate_totems[0][1] + gate_totems[1][1]) / 2),
                        ((gate_totems[1][0] + gate_totems[2][0]) / 2, (gate_totems[1][1] + gate_totems[2][1]) / 2),
                        ((gate_totems[2][0] + gate_totems[3][0]) / 2, (gate_totems[2][1] + gate_totems[3][1]) / 2)]
        gates_line = self.line(gate_centers[0], gate_centers[2])

        defer.returnValue((gate_centers, gates_line))

    @util.cancellableInlineCallbacks
    def get_pinger_line(self):
        use_hydrophones = False

        if use_hydrophones:
            # Convert heading and hydophones from to enu
            heading = yield self.pinger_heading.get_next_message()
            hydrophones_to_enu = yield self.tf_listener.get_transform('enu', heading.header.frame_id)
            hydrophones_origin = hydrophones_to_enu._p[0:2]
            heading = rosmsg_to_numpy(heading.vector)
            heading_enu = hydrophones_to_enu.transform_vector(heading)
            heading_enu = heading_enu[0:2] / np.linalg.norm(heading_enu[0:2])
            pinger_line = self.line(hydrophones_origin, hydrophones_origin + heading_enu)
        else:
            boat_pose = yield self.tx_pose
            boat_pose = boat_pose[0]
            gate1 = [65, -47, 0]
            gate2 = [54.6, -50]
            gate3 = [45, -52.6, 0]
            pinger_line = self.line([boat_pose[0], boat_pose[1], boat_pose[2]], gate3)

        defer.returnValue(pinger_line)

    def determine_pinger_gate(self, gate_centers, gates_line, pinger_line):
        intersection = self.intersection(gates_line, pinger_line)

        distances = []
        for gate_center in gate_centers:
            distances.append((gate_center[0] - intersection[0]) ** 2 + (gate_center[1] - intersection[1]) ** 2)
        argmin = np.argmin(np.array(distances))

        return argmin

    @util.cancellableInlineCallbacks
    def get_traversal_points(self, gates_line, pinger_gate, center_gate):
        traversal_vector = self.perpendicular(gates_line)
        # print(gates_line)
        # print('trav line' + str(traversal_vector))

        boat_pose = yield self.tx_pose
        boat_pose = boat_pose[0]

        traversal_distance = 7
        recenter_distance = 15
        traversal_points = [(pinger_gate[0] + traversal_vector[0] * traversal_distance,
                             pinger_gate[1] + traversal_vector[1] * traversal_distance),
                            (pinger_gate[0] + traversal_vector[0] * -traversal_distance,
                             pinger_gate[1] + traversal_vector[1] * -traversal_distance)]

        if (traversal_points[0][0] - boat_pose[0]) ** 2 + (traversal_points[0][1] - boat_pose[1]) ** 2 > (
                traversal_points[1][0] - boat_pose[0]) ** 2 + \
                (traversal_points[1][1] - boat_pose[1]) ** 2:
            traversal_points = [traversal_points[1], traversal_points[0]]

        recenter_points = [(center_gate[0] + traversal_vector[0] * recenter_distance,
                            center_gate[1] + traversal_vector[1] * recenter_distance),
                           (center_gate[0] + traversal_vector[0] * -recenter_distance,
                            center_gate[1] + traversal_vector[1] * -recenter_distance)]

        if (recenter_points[0][0] - boat_pose[0]) ** 2 + (recenter_points[0][1] - boat_pose[1]) ** 2 < \
                (recenter_points[1][0] - boat_pose[0]) ** 2 + \
                (recenter_points[1][1] - boat_pose[1]) ** 2:
            recenter_points = [recenter_points[1], recenter_points[0]]

        goal_points = [traversal_points[0], traversal_points[1], recenter_points[0]]
        goal_points_np = []

        for goal_point in goal_points:
            point = np.array(goal_point)
            point = np.append(point, [0])
            goal_points_np.append(point)

        defer.returnValue(goal_points_np)
