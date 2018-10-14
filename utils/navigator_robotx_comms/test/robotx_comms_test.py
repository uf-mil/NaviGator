#!/usr/bin/env python

import socket
import signal
import sys
import rospy

if __name__ == "__main__":

    msg_delimiter = ','

    TCP_IP = rospy.get_param('td_ip')
    # TCP_IP = '192.168.37.234'
    # TCP_PORT = rospy.get_param('td_port')
    TCP_PORT = 1337
    BUFFER_SIZE = 1024

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((TCP_IP, TCP_PORT))
    server.listen(5)

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

    while True:
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