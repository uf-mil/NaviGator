#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import navigator_tools

from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose

rospy.init_node("ogrid_gen")

class DrawGrid(object):
    def __init__(self):
        cv2.namedWindow('image')
        cv2.setMouseCallback('image', self.do_draw)
        self.clear_screen()
        self.drawing = 0

    def clear_screen(self):
        self.img = np.zeros((671, 671), np.uint8)

    def do_draw(self, event, x, y, flags, param):
        draw_vals = {1: 100, 2: 0}

        if event == cv2.EVENT_LBUTTONUP or event == cv2.EVENT_RBUTTONUP:
            self.drawing = 0
        elif event == cv2.EVENT_LBUTTONDOWN:
            self.drawing = 1
        elif event == cv2.EVENT_RBUTTONDOWN:
            self.drawing = 2
        elif self.drawing != 0:
            cv2.circle(self.img, (x, y), 5, draw_vals[self.drawing], -1)

class OGridPub(object):
    def __init__(self):
        self.grid_drawer = DrawGrid()
        self.ogrid_pub = rospy.Publisher("/ogrid", OccupancyGrid, queue_size=1)

        m = MapMetaData()
        m.resolution = 0.3
        m.width = 671
        m.height = 671
        pos = np.array([-67.5, -111.5, 1.6])
        quat = np.array([0, 0, 0, 1])
        m.origin = navigator_tools.numpy_quat_pair_to_pose(pos, quat)
        self.map_meta_data = m

        rospy.Timer(rospy.Duration(1), self.pub_grid)

    def pub_grid(self, *args):
        grid = self.grid_drawer.img

        ogrid = OccupancyGrid()
        ogrid.header = navigator_tools.make_header(frame='enu')
        ogrid.info = self.map_meta_data
        ogrid.data = np.subtract(np.flipud(grid).flatten(), 1).astype(np.int8).tolist()

        self.ogrid_pub.publish(ogrid)

o = OGridPub()

while True:
    cv2.imshow("image", o.grid_drawer.img)
    k = cv2.waitKey(100) & 0xFF

    # q to clear screen, esc to cancel out
    if k == 27:
        break
    elif k == 113:
        o.grid_drawer.clear_screen()

cv2.destroyAllWindows()