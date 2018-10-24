#!/usr/bin/env python

import socket
import signal
import sys
import time
from multiprocessing import Process

import rospy
from navigator_msgs.srv import *


def test_service():
    while not rospy.is_shutdown():
        send_robot_x_identify_symbols_dock_message("R", "TRIAN")
        time.sleep(1)
        send_robot_x_detect_deliver_message("R", "CIRCL")
        time.sleep(1)
        send_robot_x_entrance_exit_gate_message(1, 2, True, "RGB")
        time.sleep(1)
        send_robot_x_scan_code_message("RGB")
        time.sleep(1)


if __name__ == "__main__":

    msg_delimiter = ','

    TCP_IP = rospy.get_param('td_ip')
    TCP_PORT = rospy.get_param('td_port')
    # TCP_IP = '192.168.37.234'
    # TCP_PORT = 1337
    BUFFER_SIZE = 1024

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((TCP_IP, TCP_PORT))
    server.listen(5)

    rospy.wait_for_service('entrance_exit_gate_message')
    rospy.wait_for_service('scan_code_message')
    rospy.wait_for_service('identify_symbols_dock_message')
    rospy.wait_for_service('detect_deliver_message')

    send_robot_x_entrance_exit_gate_message = rospy.ServiceProxy('entrance_exit_gate_message',
                                                                 MessageExtranceExitGate)
    send_robot_x_scan_code_message = rospy.ServiceProxy('scan_code_message', MessageScanCode)
    send_robot_x_identify_symbols_dock_message = rospy.ServiceProxy('identify_symbols_dock_message',
                                                                    MessageIdentifySymbolsDock)
    send_robot_x_detect_deliver_message = rospy.ServiceProxy('detect_deliver_message',
                                                             MessageDetectDeliver)

    conn, addr = server.accept()
    print 'Connection address: ', addr

    def signal_handler(sig, frame):
        print 'You pressed Ctrl+C!'
        conn.close()
        server.close()
        sys.exit(1)


    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    timesRan = 0

    p = Process(target=test_service, args=())
    p.start()

    while not rospy.is_shutdown():
        data = conn.recv(BUFFER_SIZE)  # type: str
        if data:
            data_list = data.split(msg_delimiter)
            checksum_list = data.split("*")
            checksum = checksum_list[1]
            tot_checksum = 0
            full_data_for_checksum = checksum_list[0][1:]
            for unit_counter in range(len(full_data_for_checksum)):
                tot_checksum = ord(full_data_for_checksum[unit_counter]) ^ tot_checksum
            calc_checksum = str(tot_checksum).zfill(2) + '\r\n'
            good_checksum = checksum == calc_checksum
            if data_list[0] == '$RXHRB':
                timesRan += 1
                print 'Print #:', timesRan
                print 'Checksum Validity:', good_checksum
                print 'Message ID:', data_list[0]
                print 'HST Date:', data_list[1]
                print 'HST Time:', data_list[2]
                print 'Latitude:', data_list[3]
                print 'N/S Indicator:', data_list[4]
                print 'Longitude:', data_list[5]
                print 'E/W Indicator:', data_list[6]
                print 'Team ID:', data_list[7]
                print 'System Mode:', data_list[8]
                print 'AUV Status * Checksum:', data_list[9]
            elif data_list[0] == '$RXGAT':
                timesRan += 1
                print 'Print #:', timesRan
                print 'Checksum Validity:', good_checksum
                print 'Message ID:', data_list[0]
                print 'HST Date:', data_list[1]
                print 'HST Time:', data_list[2]
                print 'Team ID:', data_list[3]
                print 'Active Entrance Gate:', data_list[4]
                print 'Active Exit Gate:', data_list[5]
                print 'Light Buoy Active:', data_list[6]
                print 'Light Pattern * Checksum:', data_list[7]
            elif data_list[0] == '$RXCOD':
                timesRan += 1
                print 'Print #:', timesRan
                print 'Checksum Validity:', good_checksum
                print 'Message ID:', data_list[0]
                print 'HST Date:', data_list[1]
                print 'HST Time:', data_list[2]
                print 'Team ID:', data_list[3]
                print 'Light Pattern * Checksum:', data_list[4]
            elif data_list[0] == '$RXDOK':
                timesRan += 1
                print 'Print #:', timesRan
                print 'Checksum Validity:', good_checksum
                print 'Message ID:', data_list[0]
                print 'HST Date:', data_list[1]
                print 'HST Time:', data_list[2]
                print 'Team ID:', data_list[3]
                print 'Shape Color:', data_list[4]
                print 'Shape * Checksum:', data_list[5]
            elif data_list[0] == '$RXDEL':
                timesRan += 1
                print 'Print #:', timesRan
                print 'Checksum Validity:', good_checksum
                print 'Message ID:', data_list[0]
                print 'HST Date:', data_list[1]
                print 'HST Time:', data_list[2]
                print 'Team ID:', data_list[3]
                print 'Shape Color:', data_list[4]
                print 'Shape * Checksum:', data_list[5]
