#!/usr/bin/env python

'''
Keyboard Client: The keyboard client connects to the keyboard server in order
to control NaviGator using a remote keyboard. It is able to lock control of the
keyboard service to itself by obtaining a UUID that identifies it's service
calls. Curses is used to display a basic UI in the terminal that gives the user
useful feedback and captures key presses to be sent to the server.
'''


import curses

from navigator_msgs.srv import KeyboardControl
import rospy


__author__ = "Anthony Olive"
__maintainer__ = "Anthony Olive"
__email__ = "anthony@iris-systems.net"
__copyright__ = "Copyright 2016, MIL"
__license__ = "MIT"


rospy.init_node('keyboard_client', anonymous=True)


class KeyboardClient():

    def __init__(self, stdscr):
        self.screen = stdscr
        self.num_lines = 10
        self.screen.nodelay(True)
        curses.curs_set(0)

        self.uuid = ''
        self.is_locked = False

        self.keyboard_server = rospy.ServiceProxy('/keyboard_control', KeyboardControl)

        doc_strings = ["Toggle Kill:          k",
                       "Force Kill:           K",
                       "Station Hold:         h",
                       "Autonomous Control:   u",
                       "RC Control:           r",
                       "Keyboard Control:     b",
                       "Cycle Control Device: c",
                       "Move Forward:         w",
                       "Move Backward:        s",
                       "Move Port:            a",
                       "Move Starboard:       d",
                       "Yaw Counterclockwise: arrow up",
                       "Yaw Clockwise:        arrow down"
                       ]

    def read_key(self):
        keycode = -1
        new_keycode = self.screen.getch()

        while(new_keycode != -1):
            keycode = new_keycode
            new_keycode = self.screen.getch()

        if (keycode == ord('q')):
            rospy.signal_shutdown("The user has closed the keyboard client")

        return keycode if keycode != -1 else None

    def send_key(self, event):
        '''
        Sends the key to the keyboard server and stores the returned locked
        status and generated UUID (if one was received).
        '''
        keycode = self.read_key()
        service_reply = self.keyboard_server(self.uuid, keycode)

        # Flashes the interface if the locked state has changed
        if (self.is_locked != service_reply.is_locked):
            self.flash()
            self.refresh_status_text()

        self.is_locked = service_reply.is_locked
        if (service_reply.generated_uuid != ''):
            self.uuid = service_reply.generated_uuid
            self.refresh_status_text()

    def refresh_status_text(self):
        print "test"

    def write_line(self, line_num, message):
        if line_num < 0 or line_num >= self._num_lines:
            raise ValueError('line out of bounds')
        height, width = self.screen.getmaxyx()
        y = (height / self.num_lines) * line_num
        x = 2
        for text in message.split('\n'):
            text = text.ljust(width)
            self._screen.addstr(y, x, text)
            y += 1

    def refresh(self):
        self._screen.refresh()

    def flash(self):
        curses.flash()

    def clear(self):
        self.screen.clear()


def main(stdscr):
    rospy.wait_for_service('/keyboard_control')
    tele = KeyboardClient(stdscr)
    rospy.Timer(rospy.Duration(0.05), tele.send_key, oneshot=False)
    rospy.spin()

if __name__ == '__main__':
    try:
        curses.wrapper(main)
    except rospy.ROSInterruptException:
        pass
