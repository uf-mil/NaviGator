#!/usr/bin/env python

"""
RobotX Communications: A node that handles unit tests for the the RobotX Communication Protocol
"""

import socket
import traceback
import unittest
import time
import threading

from mil_tools import thread_lock
from navigator_robotx_comms.navigator_robotx_comms import *

lock = threading.Lock()


class TestRobotXComms(unittest.TestCase):

    def __init__(self, *args):
        # define delimiter for messages
        self.delim = ','
        self.team_id = rospy.get_param("~team_id")
        self.td_ip = rospy.get_param("~td_ip")
        self.td_port = rospy.get_param("~td_port")
        self.server = RobotXServer(self.td_ip, self.td_port)
        super(TestRobotXComms, self).__init__(*args)

    def test_heartbeat_message(self):
        times_ran = 0
        times_to_run = 5
        self.server.connect()

        while not rospy.is_shutdown() and times_ran < times_to_run:
            rx_data = None
            while rx_data is None:
                rx_data = self.server.receive_message()
            robotx_heartbeat_message = RobotXHeartbeatMessage()
            deserialized_msg = robotx_heartbeat_message.from_string(self.delim, rx_data)
            data_list = deserialized_msg[0]
            checksum_list = deserialized_msg[1]
            if data_list[0] == "$RXHRB":
                full_data_for_checksum = checksum_list[0].replace("$", "")
                checksum_calc = BitwiseXORChecksum()
                tot_checksum = checksum_calc.ret_checksum(full_data_for_checksum)
                final_checksum_string = str(tot_checksum).zfill(2) + "\r\n"
                self.assertEquals(checksum_list[1], final_checksum_string, "heartbeat message checksum incorrect")
                self.assertEquals(len(data_list), 10, "heartbeat message formatting incorrect")
                self.assertEquals(data_list[7], self.team_id, "team id incorrect")
                times_ran += 1
                time.sleep(1)
        self.server.conn.close()
        self.server.socket_connection.close()

    def test_entrance_exit_gate_message(self):
        times_ran = 0
        times_to_run = 5
        self.server.connect()
        entrance_gate = 1
        exit_gate = 2
        light_buoy_active = True
        light_pattern = "RBG"

        rospy.wait_for_service("entrance_exit_gate_message")
        send_robot_x_entrance_exit_gate_message = rospy.ServiceProxy("entrance_exit_gate_message",
                                                                     MessageExtranceExitGate)

        while not rospy.is_shutdown() and times_ran < times_to_run:
            rx_data = None
            send_robot_x_entrance_exit_gate_message(entrance_gate, exit_gate, light_buoy_active, light_pattern)
            while rx_data is None:
                rx_data = self.server.receive_message()
            robot_x_entrance_exit_gate_message = RobotXEntranceExitGateMessage()
            deserialized_msg = robot_x_entrance_exit_gate_message.from_string(self.delim, rx_data)
            data_list = deserialized_msg[0]
            checksum_list = deserialized_msg[1]
            if data_list[0] == "$RXGAT":
                full_data_for_checksum = checksum_list[0].replace("$", "")
                checksum_calc = BitwiseXORChecksum()
                tot_checksum = checksum_calc.ret_checksum(full_data_for_checksum)
                final_checksum_string = str(tot_checksum).zfill(2) + "\r\n"
                self.assertEquals(checksum_list[1], final_checksum_string, "entrance exit gate message checksum incorrect")
                self.assertEquals(len(data_list), 8, "entrance exit gate message formatting incorrect")
                self.assertEquals(data_list[3], self.team_id, "team id incorrect")
                self.assertEquals(int(data_list[4]), entrance_gate, "entrance gate incorrect")
                self.assertEquals(int(data_list[5]), exit_gate, "exit gate incorrect")
                if light_buoy_active:
                    self.assertEquals("Y", data_list[6], "light buoy boolean incorrect")
                else:
                    self.assertEquals("N", data_list[6], "light buoy boolean incorrect")
                msg_light_pattern = data_list[7].split("*")[0]
                self.assertEquals(light_pattern, msg_light_pattern, "light pattern incorrect")
                times_ran += 1
                time.sleep(1)
        self.server.conn.close()
        self.server.socket_connection.close()

    def test_scan_code_message(self):
        times_ran = 0
        times_to_run = 5
        self.server.connect()
        light_pattern = "RBG"

        rospy.wait_for_service("scan_code_message")
        send_robot_x_scan_code_message = rospy.ServiceProxy("scan_code_message", MessageScanCode)

        while not rospy.is_shutdown() and times_ran < times_to_run:
            rx_data = None
            send_robot_x_scan_code_message(light_pattern)
            while rx_data is None:
                rx_data = self.server.receive_message()
            robot_x_scan_code_message = RobotXScanCodeMessage()
            deserialized_msg = robot_x_scan_code_message.from_string(self.delim, rx_data)
            data_list = deserialized_msg[0]
            checksum_list = deserialized_msg[1]
            if data_list[0] == "$RXCOD":
                full_data_for_checksum = checksum_list[0].replace("$", "")
                checksum_calc = BitwiseXORChecksum()
                tot_checksum = checksum_calc.ret_checksum(full_data_for_checksum)
                final_checksum_string = str(tot_checksum).zfill(2) + "\r\n"
                self.assertEquals(checksum_list[1], final_checksum_string, "scan code message checksum incorrect")
                self.assertEquals(len(data_list), 5, "scan code message formatting incorrect")
                self.assertEquals(data_list[3], self.team_id, "team id incorrect")
                msg_light_pattern = data_list[4].split("*")[0]
                self.assertEquals(light_pattern, msg_light_pattern, "light pattern incorrect")
                times_ran += 1
                time.sleep(1)
        self.server.conn.close()
        self.server.socket_connection.close()

    def test_identify_symbols_dock_message(self):
        times_ran = 0
        times_to_run = 5
        self.server.connect()
        shape_color = "R"
        shape = "TRIAN"

        rospy.wait_for_service("identify_symbols_dock_message")
        send_robot_x_identify_symbols_dock_message = rospy.ServiceProxy("identify_symbols_dock_message",
                                                                        MessageIdentifySymbolsDock)

        while not rospy.is_shutdown() and times_ran < times_to_run:
            rx_data = None
            send_robot_x_identify_symbols_dock_message(shape_color, shape)
            while rx_data is None:
                rx_data = self.server.receive_message()
            robot_x_identify_symbols_dock_message = RobotXIdentifySymbolsDockMessage()
            deserialized_msg = robot_x_identify_symbols_dock_message.from_string(self.delim, rx_data)
            data_list = deserialized_msg[0]
            checksum_list = deserialized_msg[1]
            if data_list[0] == "$RXDOK":
                full_data_for_checksum = checksum_list[0].replace("$", "")
                checksum_calc = BitwiseXORChecksum()
                tot_checksum = checksum_calc.ret_checksum(full_data_for_checksum)
                final_checksum_string = str(tot_checksum).zfill(2) + "\r\n"
                self.assertEquals(checksum_list[1], final_checksum_string, "identify symbols dock message checksum incorrect")
                self.assertEquals(len(data_list), 6, "identify symbols dock message formatting incorrect")
                self.assertEquals(data_list[3], self.team_id, "team id incorrect")
                self.assertEquals(data_list[4], shape_color, "shape color incorrect")
                msg_shape = data_list[5].split("*")[0]
                self.assertEquals(shape, msg_shape, "shape incorrect")
                times_ran += 1
                time.sleep(1)
        self.server.conn.close()
        self.server.socket_connection.close()

    def test_detect_deliver_message(self):
        times_ran = 0
        times_to_run = 5
        self.server.connect()
        shape_color = "R"
        shape = "CIRCL"

        rospy.wait_for_service("detect_deliver_message")
        send_robot_x_detect_deliver_message = rospy.ServiceProxy("detect_deliver_message",
                                                                 MessageDetectDeliver)

        while not rospy.is_shutdown() and times_ran < times_to_run:
            rx_data = None
            send_robot_x_detect_deliver_message(shape_color, shape)
            while rx_data is None:
                rx_data = self.server.receive_message()
            robot_x_detect_deliver_message = RobotXDetectDeliverMessage()
            deserialized_msg = robot_x_detect_deliver_message.from_string(self.delim, rx_data)
            data_list = deserialized_msg[0]
            checksum_list = deserialized_msg[1]
            if data_list[0] == "$RXDEL":
                full_data_for_checksum = checksum_list[0].replace("$", "")
                checksum_calc = BitwiseXORChecksum()
                tot_checksum = checksum_calc.ret_checksum(full_data_for_checksum)
                final_checksum_string = str(tot_checksum).zfill(2) + "\r\n"
                self.assertEquals(checksum_list[1], final_checksum_string,
                                  "detect deliver message checksum incorrect")
                self.assertEquals(len(data_list), 6, "detect deliver message formatting incorrect")
                self.assertEquals(data_list[3], self.team_id, "team id incorrect")
                self.assertEquals(data_list[4], shape_color, "shape color incorrect")
                msg_shape = data_list[5].split("*")[0]
                self.assertEquals(shape, msg_shape, "shape incorrect")
                times_ran += 1
                time.sleep(1)
        self.server.conn.close()
        self.server.socket_connection.close()

class RobotXServer:
    """
    Handles communication with client for testing
    """

    def __init__(self, tcp_ip, tcp_port):
        self.tcp_ip = tcp_ip
        self.tcp_port = tcp_port
        self.connected = False
        self.socket_connection = None
        self.buffer_size = 1024
        self.conn = None
        self.conn_ip = None

    def connect(self):
        while not self.connected and not rospy.is_shutdown():
            try:
                self.socket_connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket_connection.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.socket_connection.bind((self.tcp_ip, self.tcp_port))
                self.socket_connection.listen(5)
                self.conn, self.conn_ip = self.socket_connection.accept()
                self.connected = True
            except socket.error:
                traceback.print_exc()
                self.socket_connection.close()
                time.sleep(2)

    @thread_lock(lock)
    def receive_message(self):
        while not rospy.is_shutdown():
            try:
                rx_msg = self.conn.recv(self.buffer_size)
                return rx_msg
            except socket.error:
                self.conn.close()
                self.socket_connection.close()
                self.connected = False
                self.connect()


if __name__ == "__main__":
    rospy.init_node('robotx_comms_server', anonymous=True)
    import rostest
    rostest.rosrun("robotx_comms", "robotx_comms_server", TestRobotXComms)
    unittest.main()