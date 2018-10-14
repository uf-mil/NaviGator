#!/usr/bin/env python

"""
RobotX Communications: A node that handles communications
with the RobotX Technical Director network.
"""

import datetime
import math
import socket
import time

import rospy
import tf.transformations as trans
from geometry_msgs.msg import PointStamped
from mil_tools import rosmsg_to_numpy
from nav_msgs.msg import Odometry
from navigator_msgs.srv import *

rospy.init_node("robotx_comms")

delim = ','


class RobotXHeartbeatMessage():
    """
    Handles formation and sending of heartbeat message.
    """

    def __init__(self):

        self.message_id = "RXHRB"
        self.gps_array = None
        self.odom = None
        self.auv_status = 1
        self.system_mode = 2

        rospy.Subscriber("lla", PointStamped, self.gps_coord_callback)
        rospy.Subscriber("odom", Odometry, self.gps_odom_callback)
        # rospy.Subscriber("auv_status", String, self.auv_status_callback)

    def from_string(self, string):

        # TODO: Implement testing method for heartbeat message
        pass

    def to_string(self):

        global delim
        global team_id
        global get_hst_date_time

        hst_date_time = get_hst_date_time()

        if self.gps_array is not None:
            latitude = self.gps_array.point.x
            longitude = self.gps_array.point.y
        else:
            latitude = ""
            longitude = ""

        if self.odom is not None:
            quaternion = self.odom.pose.pose.orientation
            quaternion_numpy = rosmsg_to_numpy(quaternion)
            euler_angles = trans.euler_from_quaternion(quaternion_numpy)
            north_south = ""
            east_west = ""

            if euler_angles[2] > 0 and euler_angles[2] < math.pi / 2:
                north_south = "N"
                east_west = "E"
            elif euler_angles[2] > math.pi / 2 and euler_angles[2] < math.pi:
                north_south = "N"
                east_west = "W"
            elif euler_angles[2] < -math.pi / 2 and euler_angles[2] > -math.pi:
                north_south = "S"
                east_west = "W"
            elif euler_angles[2] < 0 and euler_angles[2] > -math.pi / 2:
                north_south = "S"
                east_west = "E"

        # TODO: EULER ANGLES pi to -pi, in radians, get degrees, fix orientiation.

        else:
            east_west = ""
            north_south = ""

        first_half_data = "{0}{1}{2}{3}{4}{5}{6}".format(self.message_id,
                                                         delim,
                                                         hst_date_time,
                                                         delim,
                                                         latitude,
                                                         delim,
                                                         north_south)

        second_half_data = "{0}{1}{2}{3}{4}{5}{6}{7}{8}".format(longitude,
                                                                delim,
                                                                east_west,
                                                                delim,
                                                                team_id,
                                                                delim, str(self.system_mode), delim,
                                                                str(self.auv_status))

        full_data = first_half_data + delim + second_half_data

        # Test data
        # full_data = 'RXHRB,101218,161229,21.31198,N,157.88972,W,AUVSI,2,1'

        tot_checksum = 0
        for unitCounter in range(len(full_data)):
            tot_checksum = ord(full_data[unitCounter]) ^ tot_checksum

        checksum = tot_checksum

        msg_return = "${0}*{1}\r\n".format(full_data, str(checksum).zfill(2))

        return msg_return

    def auv_status_callback(self, auv_status):

        self.auv_status = auv_status

    def gps_coord_callback(self, lla):

        self.gps_array = lla

    def gps_odom_callback(self, odom):

        self.odom = odom


class RobotXEntranceExitGateMessage():
    """
    Handles formation and sending of entrance and exit gates message
    """

    def __init__(self):

        self.message_id = "RXGAT"

    def from_string(self, string):

        # TODO: Implement testing method for entrance and exit gates message
        pass

    def to_string(self, data):

        global delim
        global team_id
        global get_hst_date_time

        hst_date_time = get_hst_date_time()  # type: str

        light_buoy_active_letter = "N"

        if data.light_buoy_active:
            light_buoy_active_letter = "Y"

        data = "{0}{1}{2}{3}{4}{5}{6}{7}{8}{9}{10}{11}{12}".format(self.message_id,
                                                                   delim,
                                                                   hst_date_time,
                                                                   delim,
                                                                   team_id,
                                                                   delim,
                                                                   str(data.entrance_gate),
                                                                   delim,
                                                                   str(data.exit_gate),
                                                                   delim,
                                                                   light_buoy_active_letter,
                                                                   delim,
                                                                   data.light_pattern, )

        # Test data
        # full_data = '$RXGAT,101218,161229,AUVSI,1,2,Y,RBG*25'

        tot_checksum = 0
        for unitCounter in range(len(data)):
            tot_checksum = ord(data[unitCounter]) ^ tot_checksum

        checksum = tot_checksum

        msg_return = "${0}*{1}\r\n".format(data, str(checksum).zfill(2))

        send_message(msg_return)

        return MessageExtranceExitGateResponse(msg_return)


class RobotXScanCodeMessage():
    """
    Handles formation and sending of scan the code message
    """

    def __init__(self):
        self.message_id = "RXCOD"

    def from_string(self, string):
        # TODO: Implement testing method for scan the code message
        pass

    def to_string(self, data):
        global delim
        global team_id
        global get_hst_date_time

        hst_date_time = get_hst_date_time()  # type: str

        data = "{0}{1}{2}{3}{4}{5}{6}".format(self.message_id,
                                              delim,
                                              hst_date_time,
                                              delim,
                                              team_id,
                                              delim,
                                              data.light_pattern)

        # Test data
        # full_data = '$RXCOD,101218,161229,AUVSI,RBG*49'

        tot_checksum = 0
        for unitCounter in range(len(data)):
            tot_checksum = ord(data[unitCounter]) ^ tot_checksum

        checksum = tot_checksum

        msg_return = "${0}*{1}\r\n".format(data, str(checksum).zfill(2))

        send_message(msg_return)

        return MessageScanCodeResponse(msg_return)


class RobotXIdentifySymbolsDockMessage():
    """
    Handles formation and sending of identify symbols and dock message
    """

    def __init__(self):
        self.message_id = "RXDOK"

    def from_string(self, string):
        # TODO: Implement testing method for identify symbols and dock message
        pass

    def to_string(self, data):
        global delim
        global team_id
        global get_hst_date_time

        hst_date_time = get_hst_date_time()  # type: str

        data = "{0}{1}{2}{3}{4}{5}{6}{7}{8}".format(self.message_id,
                                                    delim,
                                                    hst_date_time,
                                                    delim,
                                                    team_id,
                                                    delim,
                                                    data.shape_color,
                                                    delim,
                                                    data.shape)

        # Test data
        # full_data = 'RXDOK,101218,161229,AUVSI,R,TRIAN*28'

        tot_checksum = 0
        for unitCounter in range(len(data)):
            tot_checksum = ord(data[unitCounter]) ^ tot_checksum

        checksum = tot_checksum

        msg_return = "${0}*{1}\r\n".format(data, str(checksum).zfill(2))

        send_message(msg_return)

        return MessageIdentifySymbolsDockResponse(msg_return)


class RobotXDetectDeliverMessage():
    """
    Handles formation and sending of detect and deliver message
    """

    def __init__(self):
        self.message_id = "RXDEL"

    def from_string(self, string):
        # TODO: Implement testing method for detect and deliver message
        pass

    def to_string(self, data):
        global delim
        global team_id
        global get_hst_date_time

        hst_date_time = get_hst_date_time()  # type: str

        data = "{0}{1}{2}{3}{4}{5}{6}{7}{8}".format(self.message_id,
                                                    delim,
                                                    hst_date_time,
                                                    delim,
                                                    team_id,
                                                    delim,
                                                    data.shape_color,
                                                    delim,
                                                    data.shape)

        # Test data
        # full_data = 'RXDEL,101218,161229,AUVSI,R,CIRCL*32'

        tot_checksum = 0
        for unitCounter in range(len(data)):
            tot_checksum = ord(data[unitCounter]) ^ tot_checksum

        checksum = tot_checksum

        msg_return = "${0}*{1}\r\n".format(data, str(checksum).zfill(2))

        send_message(msg_return)

        return MessageDetectDeliverResponse(msg_return)


def get_hst_date_time():
    # HST is 10 hours behind UTC
    hst_time = datetime.datetime.utcnow() - datetime.timedelta(hours=10, minutes=0)
    date_string = hst_time.strftime("%d%m%y")
    time_string = hst_time.strftime("%H%M%S")
    return date_string + delim + time_string


def send_message(message):
    global TCP_IP
    global TCP_PORT
    global socketConnection
    global connected
    while not rospy.is_shutdown():
        try:
            socketConnection.send(message)
            break
        except socket.error:
            print('Connection to TD Server Lost, Attempting Reconnection')
            connected = False
            # recreate socket
            socketConnection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            while not connected:
                # attempt to reconnect, otherwise sleep for 2 seconds
                try:
                    socketConnection.connect((TCP_IP, TCP_PORT))
                    connected = True
                    print("Re-connection to TD Server Successful")
                except socket.error:
                    time.sleep(2)


if __name__ == "__main__":

    # INIT TCP
    TCP_IP = rospy.get_param('td_ip')
    TCP_PORT = rospy.get_param('td_port')
    BUFFER_SIZE = 1024
    connected = False
    socketConnection = None

    # Constants used in multiple messages
    # team_id = rospy.get_param(team_id)
    team_id = "GATOR"

    robot_x_heartbeat_message = RobotXHeartbeatMessage()

    robot_x_entrance_exit_gate_message = RobotXEntranceExitGateMessage()
    robot_x_scan_code_message = RobotXScanCodeMessage()
    robot_x_identify_symbols_dock_message = RobotXIdentifySymbolsDockMessage()
    robot_x_detect_deliver_message = RobotXDetectDeliverMessage()

    service_entrance_exit_gate_message = rospy.Service('entrance_exit_gate_message', MessageExtranceExitGate,
                                                       robot_x_entrance_exit_gate_message.to_string)
    service_scan_code_message = rospy.Service('scan_code_message', MessageScanCode, robot_x_scan_code_message.to_string)
    service_identify_symbols_dock_message = rospy.Service('identify_symbols_dock_message', MessageIdentifySymbolsDock,
                                                          robot_x_identify_symbols_dock_message.to_string)
    service_detect_deliver_message = rospy.Service('detect_deliver_message', MessageDetectDeliver,
                                                   robot_x_detect_deliver_message.to_string)

    while not rospy.is_shutdown():
        if not connected:
            print('Attempting Connection to TD Server')
        while not connected:
            connected = False
            # recreate socket
            socketConnection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            # attempt to reconnect, otherwise sleep for 2 seconds
            try:
                socketConnection.connect((TCP_IP, TCP_PORT))
                connected = True
                print('Connection to TD Server Successful')
            except socket.error:
                time.sleep(2)
        while not rospy.is_shutdown():
            print('Sending Heartbeat!')
            heartbeat_msg = robot_x_heartbeat_message.to_string()
            send_message(heartbeat_msg)
            time.sleep(1)

    if rospy.is_shutdown():
        socketConnection.close()

    rospy.spin()
