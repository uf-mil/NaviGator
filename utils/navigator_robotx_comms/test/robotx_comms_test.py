#!/usr/bin/env python

import socket
import signal
import sys
import rospy

msg_delimiter = ","

TCP_IP = rospy.get_param('td_ip')
TCP_PORT = rospy.get_param('td_port')
BUFFER_SIZE = 1024

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind((TCP_IP, TCP_PORT))
server.listen(5)

conn, addr = server.accept()
print 'Connection address: ', addr


def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    conn.close()
    server.close()
    sys.exit(1)


signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

timesRan = 0

while True:
    if timesRan >= 100:
        print 'Test Success: Heartbeat received from NaviGator 100 times.'
        sys.exit(0)
    data = conn.recv(BUFFER_SIZE)  # type: str
    if data:
        dataList = data.split(msg_delimiter)
        if dataList[0] == '$RXHRB':
            timesRan += 1
            print 'Print #:', timesRan
            print 'Message ID:', dataList[0]
            print 'HST Date:', dataList[1]
            print 'HST Time:', dataList[2]
            print 'Latitude:', dataList[3]
            print 'N/S Indicator:', dataList[4]
            print 'Longitude:', dataList[5]
            print 'E/W Indicator:', dataList[6]
            print 'Team ID:', dataList[7]
            print 'System Mode:', dataList[8]
            print 'AUV Status * Checksum:', dataList[9]