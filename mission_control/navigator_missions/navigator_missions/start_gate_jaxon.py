#!/usr/bin/env python
from txros import util
from navigator_missions.navigator import Navigator
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import PointStamped
import numpy as np
from mil_tools import rosmsg_to_numpy
from twisted.internet import defer
import math
from ros_alarms import TxAlarmBroadcaster


class StartGateJaxon(Navigator):
    @classmethod
    @util.cancellableInlineCallbacks
    def init(cls):
        cls.pinger_pos_sub = cls.nh.subscribe("/hydrophones/position", PointStamped)
        cls.alarm_broadcaster = yield TxAlarmBroadcaster.init(cls.nh, 'kill')

    @util.cancellableInlineCallbacks
    def run(self, args):
        gate_results = yield self.find_gates()
        gate_centers = gate_results[0]
        gates_line = gate_results[1]

        scan_points = yield self.get_scan_point(gates_line, gate_centers[1])
        scan_point = scan_points[0]
        scan_lookat = scan_points[1]

        yield self.move.set_position(scan_point).look_at(scan_lookat).go()

        # This is necessary because we manually adjust the alarms
        assert not self.killed, 'Cannot run when killed'

        yield self.alarm_broadcaster.raise_alarm()

        yield self.nh.sleep(3)

        pinger_pos = yield self.get_pinger_pos()

        yield self.alarm_broadcaster.clear_alarm()

        pinger_gate_index = self.determine_pinger_gate(gate_centers, pinger_pos)
        pinger_gate = gate_centers[pinger_gate_index]


        self.send_feedback(' Gate identified as ' + str(pinger_gate_index + 1))


        traversal_points = yield self.get_traversal_points(gates_line, pinger_gate, gate_centers[1])

        yield self.move.set_position(traversal_points[0]).look_at(traversal_points[1]).go()
        yield self.move.set_position(traversal_points[1]).go()

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
    def perpendicular(L):
        v1 = [L[1], -L[0], 0]
        r = v1[0] ** 2 + v1[1] ** 2
        v1 = [v1[0] / r, v1[1] / r]
        v2 = [0, 0, 1]
        pvec = np.cross(v1, v2)
        r = math.sqrt(pvec[0] ** 2 + pvec[1] ** 2)
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
        t1 = t1[0]
        white_totems = yield self.find_nearest_objects("totem_white", 2)
        t2 = white_totems[0]
        t3 = white_totems[1]
        t4 = yield self.find_nearest_objects("totem_green", 1)
        t4 = t4[0]

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
    def get_pinger_pos(self):
        use_hydrophones = False

        if use_hydrophones:
            rospos = yield self.pinger_pos_sub.get_next_message()
            pinger_pos = rosmsg_to_numpy(rospos.point)[0:2]
        else:
            gate1 = (12.5, 0)
            gate2 = (10.5, -10)
            gate3 = (7.5, -20)
            pinger_pos = gate3

        defer.returnValue(pinger_pos)

    def determine_pinger_gate(self, gate_centers, pinger_pos):
        distances = []
        for gate_center in gate_centers:
            distances.append((gate_center[0] - pinger_pos[0]) ** 2 + (gate_center[1] - pinger_pos[1]) ** 2)
        argmin = np.argmin(np.array(distances))

        return argmin

    @util.cancellableInlineCallbacks
    def get_scan_point(self, gates_line, center_gate):
        traversal_vector = self.perpendicular(gates_line)

        boat_pose = yield self.tx_pose
        boat_pose = boat_pose[0]

        recenter_distance = 15

        recenter_points = [(center_gate[0] + traversal_vector[0] * recenter_distance,
                            center_gate[1] + traversal_vector[1] * recenter_distance),
                           (center_gate[0] + traversal_vector[0] * -recenter_distance,
                            center_gate[1] + traversal_vector[1] * -recenter_distance)]

        if (recenter_points[0][0] - boat_pose[0]) ** 2 + (recenter_points[0][1] - boat_pose[1]) ** 2 > \
                (recenter_points[1][0] - boat_pose[0]) ** 2 + \
                (recenter_points[1][1] - boat_pose[1]) ** 2:
            recenter_points = [recenter_points[1], recenter_points[0]]

        recenter_points_np = []

        for goal_point in recenter_points:
            point = np.array(goal_point)
            point = np.append(point, [0])
            recenter_points_np.append(point)

        defer.returnValue(recenter_points_np)

    @util.cancellableInlineCallbacks
    def get_traversal_points(self, gates_line, pinger_gate, center_gate):
        traversal_vector = self.perpendicular(gates_line)

        boat_pose = yield self.tx_pose
        boat_pose = boat_pose[0]

        traversal_distance = 7
        traversal_points = [(pinger_gate[0] + traversal_vector[0] * traversal_distance,
                             pinger_gate[1] + traversal_vector[1] * traversal_distance),
                            (pinger_gate[0] + traversal_vector[0] * -traversal_distance,
                             pinger_gate[1] + traversal_vector[1] * -traversal_distance)]

        if (traversal_points[0][0] - boat_pose[0]) ** 2 + (traversal_points[0][1] - boat_pose[1]) ** 2 > (
                traversal_points[1][0] - boat_pose[0]) ** 2 + \
                (traversal_points[1][1] - boat_pose[1]) ** 2:
            traversal_points = [traversal_points[1], traversal_points[0]]

        traversal_points_np = []

        for goal_point in traversal_points:
            point = np.array(goal_point)
            point = np.append(point, [0])
            traversal_points_np.append(point)

        defer.returnValue(traversal_points_np)