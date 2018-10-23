#!/usr/bin/env python

"""
RobotX Communications: A node that handles communications
with the RobotX Technical Director network.
"""

import datetime
import math
import socket
import time
import threading

import rospy
import tf.transformations as trans
from geometry_msgs.msg import PointStamped
from mil_tools import rosmsg_to_numpy
from nav_msgs.msg import Odometry
from navigator_msgs.srv import *
from mil_tools import thread_lock

lock = threading.Lock()

rospy.init_node("robotx_comms")


class RobotXHeartbeatMessage():
    """
    Handles formation and sending of heartbeat message.
    """

    def __init__(self):

        self.team_id = rospy.get_param('team_id')
        self.message_id = "RXHRB"
        self.gps_array = None
        self.odom = None
        self.auv_status = 1
        self.system_mode = 2

        rospy.Subscriber("lla", PointStamped, self.gps_coord_callback)
        rospy.Subscriber("odom", Odometry, self.gps_odom_callback)
        # rospy.Subscriber("auv_status", String, self.auv_status_callback)

        # Do Class that subscribes to all stuff
        #

    def from_string(self, string):

        # TODO: Implement testing method for heartbeat message
        pass

    def to_string(self, delim, team_id, hst_date_time):

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

            if 0 < euler_angles[2] < math.pi / 2:
                north_south = "N"
                east_west = "E"
            elif math.pi / 2 < euler_angles[2] < math.pi:
                north_south = "N"
                east_west = "W"
            elif -math.pi / 2 > euler_angles[2] > -math.pi:
                north_south = "S"
                east_west = "W"
            elif 0 > euler_angles[2] > -math.pi / 2:
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

        return MessageHeartbeatResponse(msg_return)

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

    def to_string(self, delim, team_id, hst_date_time, data):

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
                                                                   data.light_pattern)

        # Test data
        # full_data = '$RXGAT,101218,161229,AUVSI,1,2,Y,RBG*25'

        tot_checksum = 0
        for unitCounter in range(len(data)):
            tot_checksum = ord(data[unitCounter]) ^ tot_checksum

        checksum = tot_checksum

        msg_return = "${0}*{1}\r\n".format(data, str(checksum).zfill(2))

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

    def to_string(self, delim, team_id, hst_date_time, data):
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

    def to_string(self, delim, team_id, hst_date_time, data):
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

    def to_string(self, delim, team_id, hst_date_time, data):
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

        return MessageDetectDeliverResponse(msg_return)


class RobotXStartServices():

    def __init__(self):
        # define delimiter for messages
        self.delim = ','
        # define team id for messages
        self.team_id = rospy.get_param('team_id')
        # initialize connection to server
        self.robot_x_client = RobotXClient()
        self.robot_x_client.connect()

        # setup all message types
        self.robot_x_heartbeat_message = RobotXHeartbeatMessage()
        self.robot_x_entrance_exit_gate_message = RobotXEntranceExitGateMessage()
        self.robot_x_scan_code_message = RobotXScanCodeMessage()
        self.robot_x_identify_symbols_dock_message = RobotXIdentifySymbolsDockMessage()
        self.robot_x_detect_deliver_message = RobotXDetectDeliverMessage()

        # setup all services
        self.service_entrance_exit_gate_message = rospy.Service('entrance_exit_gate_message',
                                                                MessageExtranceExitGate,
                                                                self.handle_entrance_exit_gate_message)
        self.service_scan_code_message = rospy.Service('scan_code_message',
                                                       MessageScanCode,
                                                       self.handle_scan_code_message)
        self.service_identify_symbols_dock_message = rospy.Service('identify_symbols_dock_message',
                                                                   MessageIdentifySymbolsDock,
                                                                   self.handle_identify_symbols_dock_message)
        self.service_detect_deliver_message = rospy.Service('detect_deliver_message',
                                                            MessageDetectDeliver,
                                                            self.handle_detect_deliver_message)

        # start sending heartbeat every second
        rospy.Timer(rospy.Duration(1), self.handle_heartbeat_message)

    def handle_heartbeat_message(self, data):
        hst_date_time = self.get_hst_date_time()
        message = self.robot_x_heartbeat_message.to_string(self.delim, self.team_id, hst_date_time)
        self.robot_x_client.send_message(message.message)
        return message

    def handle_entrance_exit_gate_message(self, data):
        hst_date_time = self.get_hst_date_time()
        message = self.robot_x_entrance_exit_gate_message.to_string(self.delim, self.team_id, hst_date_time, data)
        self.robot_x_client.send_message(message.message)
        return message

    def handle_scan_code_message(self, data):
        hst_date_time = self.get_hst_date_time()
        message = self.robot_x_scan_code_message.to_string(self.delim, self.team_id, hst_date_time, data)
        self.robot_x_client.send_message(message.message)
        return message

    def handle_identify_symbols_dock_message(self, data):
        hst_date_time = self.get_hst_date_time()
        message = self.robot_x_identify_symbols_dock_message.to_string(self.delim, self.team_id, hst_date_time, data)
        self.robot_x_client.send_message(message.message)
        return message

    def handle_detect_deliver_message(self, data):
        hst_date_time = self.get_hst_date_time()
        message = self.robot_x_detect_deliver_message.to_string(self.delim, self.team_id, hst_date_time, data)
        self.robot_x_client.send_message(message.message)
        return message

    def get_hst_date_time(self):
        # HST is 10 hours behind UTC
        hst_time = datetime.datetime.utcnow() - datetime.timedelta(hours=10, minutes=0)
        date_string = hst_time.strftime("%d%m%y")
        time_string = hst_time.strftime("%H%M%S")
        return date_string + self.delim + time_string


class RobotXClient():

    def __init__(self):
        self.TCP_IP = rospy.get_param('td_ip')
        self.TCP_PORT = rospy.get_param('td_port')
        self.BUFFER_SIZE = 1024
        self.connected = False
        self.socket_connection = None

    def recreate_socket(self):
        self.socket_connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def connect(self):
        if not self.connected:
            print('Attempting Connection to TD Server')
        while not self.connected and not rospy.is_shutdown():
            # recreate socket
            self.socket_connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            # attempt to reconnect, otherwise sleep for 2 seconds
            try:
                self.recreate_socket()
                self.connected = True
                print('Connection to TD Server Successful')
            except socket.error:
                time.sleep(2)

    @thread_lock(lock)
    def send_message(self, message):
        while not rospy.is_shutdown():
            try:
                self.socket_connection.send(message)
                break
            except socket.error:
                print('Connection to TD Server Lost, Attempting Reconnection')
                self.connected = False
                # recreate socket
                self.recreate_socket()
                while not self.connected and not rospy.is_shutdown():
                    # attempt to reconnect, otherwise sleep for 2 seconds
                    try:
                        self.socket_connection.connect((self.TCP_IP, self.TCP_PORT))
                        self.connected = True
                        print("Re-connection to TD Server Successful")
                    except socket.error:
                        time.sleep(2)


if __name__ == "__main__":

    robot_x_start_services = RobotXStartServices()
    rospy.spin()