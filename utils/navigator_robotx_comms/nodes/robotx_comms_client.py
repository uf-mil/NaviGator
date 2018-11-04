#!/usr/bin/env python

"""
RobotX Communications: A node that handles message communications
with the Technical Director server for the RobotX Communication Protocol
"""

import datetime
import socket
import threading
import time

from geometry_msgs.msg import PointStamped
from mil_tools import thread_lock
from nav_msgs.msg import Odometry

from navigator_robotx_comms.navigator_robotx_comms import *

lock = threading.Lock()

rospy.init_node("robotx_comms_client")


class RobotXStartServices:
    """
    Initializes services and subscribes to necessary publishers
    """

    def __init__(self):
        # define all variables for subscribers
        self.gps_array = None
        self.odom = None
        self.auv_status = 1  # TODO: Change to None after auv_status publisher becomes a thing
        self.system_mode = 2  # TODO: Change to None after system_mode publisher becomes a thing
        # define delimiter for messages
        self.delim = ','
        # define parameters
        self.team_id = rospy.get_param("~team_id")
        self.td_ip = rospy.get_param("~td_ip")
        self.td_port = rospy.get_param("~td_port")
        # initialize connection to server
        self.robotx_client = RobotXClient(self.td_ip, self.td_port)
        self.robotx_client.connect()

        # setup all message types
        self.robotx_heartbeat_message = RobotXHeartbeatMessage()
        self.robotx_entrance_exit_gate_message = RobotXEntranceExitGateMessage()
        self.robotx_scan_code_message = RobotXScanCodeMessage()
        self.robotx_identify_symbols_dock_message = RobotXIdentifySymbolsDockMessage()
        self.robotx_detect_deliver_message = RobotXDetectDeliverMessage()

        # setup all subscribers
        rospy.Subscriber("lla", PointStamped, self.gps_coord_callback)
        rospy.Subscriber("odom", Odometry, self.gps_odom_callback)
        # rospy.Subscriber("auv_status", String, self.auv_status_callback)  # TODO: Uncomment when auv_status publisher becomes a thing
        # rospy.Subscriber("system_mode", String, self.system_mode_callback)  # TODO: Uncomment when system_mode publisher becomes a thing

        # setup all services
        self.service_entrance_exit_gate_message = rospy.Service("entrance_exit_gate_message",
                                                                MessageExtranceExitGate,
                                                                self.handle_entrance_exit_gate_message)
        self.service_scan_code_message = rospy.Service("scan_code_message",
                                                       MessageScanCode,
                                                       self.handle_scan_code_message)
        self.service_identify_symbols_dock_message = rospy.Service("identify_symbols_dock_message",
                                                                   MessageIdentifySymbolsDock,
                                                                   self.handle_identify_symbols_dock_message)
        self.service_detect_deliver_message = rospy.Service("detect_deliver_message",
                                                            MessageDetectDeliver,
                                                            self.handle_detect_deliver_message)

        # start sending heartbeat every second
        rospy.Timer(rospy.Duration(1), self.handle_heartbeat_message)

    def gps_coord_callback(self, lla):
        self.gps_array = lla

    def gps_odom_callback(self, odom):
        self.odom = odom

    def auv_status_callback(self, auv_status):
        self.auv_status = auv_status

    def system_mode_callback(self, system_mode):
        self.system_mode = system_mode

    def handle_heartbeat_message(self, data):
        hst_date_time = self.get_hst_date_time()
        message = self.robotx_heartbeat_message.to_string(self.delim, self.team_id, hst_date_time, self.gps_array,
                                                          self.odom, self.auv_status, self.system_mode)
        self.robotx_client.send_message(message)

    def handle_entrance_exit_gate_message(self, data):
        hst_date_time = self.get_hst_date_time()
        message = self.robotx_entrance_exit_gate_message.to_string(self.delim, self.team_id, hst_date_time, data)
        self.robotx_client.send_message(message.message)
        return message

    def handle_scan_code_message(self, data):
        hst_date_time = self.get_hst_date_time()
        message = self.robotx_scan_code_message.to_string(self.delim, self.team_id, hst_date_time, data)
        self.robotx_client.send_message(message.message)
        return message

    def handle_identify_symbols_dock_message(self, data):
        hst_date_time = self.get_hst_date_time()
        message = self.robotx_identify_symbols_dock_message.to_string(self.delim, self.team_id, hst_date_time, data)
        self.robotx_client.send_message(message.message)
        return message

    def handle_detect_deliver_message(self, data):
        hst_date_time = self.get_hst_date_time()
        message = self.robotx_detect_deliver_message.to_string(self.delim, self.team_id, hst_date_time, data)
        self.robotx_client.send_message(message.message)
        return message

    def get_hst_date_time(self):
        # HST is 10 hours behind UTC
        hst_time = datetime.datetime.utcnow() - datetime.timedelta(hours=10, minutes=0)
        date_string = hst_time.strftime("%d%m%y")
        time_string = hst_time.strftime("%H%M%S")
        return date_string + self.delim + time_string


class RobotXClient:
    """
    Handles communication with Technical Director server
    """

    def __init__(self, tcp_ip, tcp_port):
        self.tcp_ip = tcp_ip
        self.tcp_port = tcp_port
        self.connected = False
        self.socket_connection = None

    def connect(self):
        if not self.connected:
            rospy.loginfo("Attempting Connection to TD Server")
        while not self.connected and not rospy.is_shutdown():
            # recreate socket
            self.socket_connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            # attempt to reconnect, otherwise sleep for 2 seconds
            try:
                self.socket_connection.connect((self.tcp_ip, self.tcp_port))
                self.connected = True
                rospy.loginfo("Connection to TD Server Successful")
            except socket.error:
                time.sleep(2)

    @thread_lock(lock)
    def send_message(self, message):
        while not rospy.is_shutdown():
            try:
                self.socket_connection.send(message)
                break
            except socket.error:
                rospy.loginfo("Connection to TD Server Lost")
                self.connected = False
                self.connect()


if __name__ == "__main__":
    robotx_start_services = RobotXStartServices()
    rospy.spin()