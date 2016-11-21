#!/usr/bin/env python

import rospy

import threading
import serial

from std_msgs.msg import String

from navigator_tools import fprint, thread_lock
from navigator_alarm import AlarmBroadcaster, AlarmListener

lock = threading.Lock()

class KillInterface(object):
    """
    This handles the comms node between ROS and kill/status embedded board.
    There are two things running here:
        1. From ROS: Check current operation mode of the boat and tell that to the light
        2. From BOARD: Check the current kill status from the other sources
    """

    def __init__(self, port="/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A104OWRY-if00-port0", baud=9600):
        self.ser = serial.Serial(port=port, baud=baud, timeout=0.25)

        self.current_wrencher = ''
        _set_wrencher = lambda msg: setattr(self, 'current_wrencher', msg.data)
        rospy.subscriber("/wrench/current", String, _set_wrencher)

        self.killed = False
        self.kill_broad = AlarmBroadcaster("kill")
        AlarmListener("kill", self.alarm_kill_cb)
    
    @thread_lock(lock)
    def alarm_kill_cb(self, alarm):
        # Ignore if the alarm doesn't change our status
        if (self.killed and not alarm.clear) or (not self.killed and alarm.clear):
            return

        self.killed = alarm.clear
        
        if alarm.clear:
            self.ser.write('\x45')
            resp = self.ser.read()
            if resp == '\x55':
                # Okay
                fprint("Kill OK")
            else:
                # Uh oh
                fprint("Error in response (got {})".format(resp))
        else:
            self.ser.write('\x46')
            resp = self.ser.read()
            if resp == '\x56':
                # Okay
                fprint("Unkill OK")
            else:
                # Uh oh
                fprint("Error in response (got {})".format(resp))
    
    @thread_lock(lock)
    def request(self, write_str, recv_str=None):
        """
        Deals with requesting data and checking if the response matches some `recv_str`.
        Returns True or False depending on the response.
        """
        self.ser.write(write_str)
        
    def control_update(self):
        # Update status light with current control
        if self.current_wrencher = '':
            

    def get_status(self):
        """
        Request an u:pdates all current status indicators
        """
        # Overall kill status
        self.ser.write('\x21')
        if self.ser.read() == '\x00':
            self.killed = False
        elif self.ser.read() == '\x01':
            self.killed = True

    @thread_lock(lock)
    def ping(self):
        fprint("Pinging...")
        self.ser.write('\x20')
        if self.ser.read() == '\x30':
            fprint("Ping response!", msg_color='green')
        else:
            fprint("No ping response found", msg_color='red')
