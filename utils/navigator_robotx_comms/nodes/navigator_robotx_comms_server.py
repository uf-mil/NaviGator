#!/usr/bin/env python

"""
RobotX Communications: A node that handles communications
with the RobotX Technical Director network.
"""

import rospy
import math
from nav_msgs.msg import Odometry
from mil_tools import rosmsg_to_numpy
from geometry_msgs.msg import PointStamped
import socket
import datetime
import time
import tf.transformations as trans

rospy.init_node("robotx_comms")


class RobotXComms():
    """
    Handles communications
    with the RobotX Technical Director network.
    """
    system_mode = None  # type: int

    def __init__(self):

        self.gps_array = None
        self.odom = None
        self.auv_status = 1
        self.system_mode = 2
        # self.TEAM_ID = rospy.get_param(team_id)
        self.team_id = "GATOR"

        # rospy.Subscriber("auv_status", String, self.auv_status_callback)

    def heartbeat_msg(self):

        message_id = "RXHRB"

        delim = ','

        # HST is 10 hours behind UTC
        hst_time = datetime.datetime.utcnow() - datetime.timedelta(hours=10, minutes=0)
        date_string = hst_time.strftime("%d%m%y")
        time_string = hst_time.strftime("%H%M%S")

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

            print(euler_angles[2])
            if euler_angles[2] > 0 and euler_angles[2] < math.pi/2:
                north_south = "N"
                east_west = "E"
            elif euler_angles[2] > math.pi/2 and euler_angles[2] < math.pi:
                north_south = "N"
                east_west = "W"
            elif euler_angles[2] < -math.pi/2 and euler_angles[2] > -math.pi:
                north_south = "S"
                east_west = "W"
            elif euler_angles[2] < 0 and euler_angles[2] > -math.pi/2:
                north_south = "S"
                east_west = "E"

        # TODO: EULER ANGLES pi to -pi, in radians, get degrees, fix orientiation.

        else:
            east_west = ""
            north_south = ""

        first_half_checksum = "{0}{1}{2}{3}{4}{5}{6}{7}{8}".format(message_id,
                                                                  delim,
                                                                  date_string,
                                                                  delim,
                                                                  time_string,
                                                                  delim,
                                                                  latitude,
                                                                  delim,
                                                                  north_south)

        second_half_checksum = "{0}{1}{2}{3}{4}{5}{6}{7}{8}".format(longitude,
                                                              delim,
                                                              east_west,
                                                              delim,
                                                              self.team_id,
                                                              delim, str(self.system_mode), delim, str(self.auv_status))

        full_data_for_checksum = first_half_checksum + delim + second_half_checksum

        # Test data
        # full_data_for_checksum = 'RXHRB,101218,161229,21.31198,N,157.88972,W,AUVSI,2,1'

        tot_checksum = 0
        for unitCounter in range (len(full_data_for_checksum)):
            tot_checksum = ord(full_data_for_checksum[unitCounter]) ^ tot_checksum

        checksum = tot_checksum

        msg_return = "${0}*{1}\r\n".format(full_data_for_checksum, str(checksum).zfill(2))

        return msg_return

    def auv_status_callback(self, auv_status):

        self.auv_status = auv_status

    def gps_coord_callback(self, lla):

        self.gps_array = lla

    def gps_odom_callback(self, odom):

        self.odom = odom

if __name__ == "__main__":

    # INIT TCP
    TCP_IP = rospy.get_param('td_ip')
    TCP_PORT = rospy.get_param('td_port')
    BUFFER_SIZE = 1024
    connected = False

    socketConnection = None

    robot_x_comms = RobotXComms()
    rospy.Subscriber("lla", PointStamped,  robot_x_comms.gps_coord_callback)
    rospy.Subscriber("odom", Odometry, robot_x_comms.gps_odom_callback)

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
        heartbeat_msg = robot_x_comms.heartbeat_msg()
        while True:
            print('Sending Heartbeat!')
            try:
                socketConnection.send(heartbeat_msg)
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
                        print( "Re-connection to TD Server Successful" )
                    except socket.error:
                        time.sleep(2)
        time.sleep(1)

    if rospy.is_shutdown():
        socketConnection.close()

    rospy.spin()