#!/usr/bin/env python
import cv2
import numpy as np
import math
import argparse
#import imutils
import rospy
from matplotlib import pyplot as plt
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import roslib
import sys
import rospkg
import os

# This program is ussed to detect the shape and color in docking task,
# active this program when the boat is close to dock symbol

# import template picture form file system
rospack = rospkg.RosPack()
dock_file = os.path.join(rospack.get_path('navigator_vision'), 'templates/dock_shape_identification/')

imgCr = cv2.imread(dock_file+'cross.png', 0)
medianCr = cv2.medianBlur(imgCr, 11)
edgeCr = cv2.Canny(medianCr, 100, 200)

imgCir = cv2.imread(dock_file+'circle.png', 0)
medianCir = cv2.medianBlur(imgCir, 11)
edgeCir = cv2.Canny(medianCir, 100, 200)


class dock_detection:

    def __init__(self):
		self.bridge = CvBridge()
		# subscribe the camera topic from ros
		self.image_sub = rospy.Subscriber(
			"/camera/front/left/image_raw", Image, self.callback)
		self.pub = rospy.Publisher('dock_result', String, queue_size=10)
		# initial value of msg to publish
		shape = None
		color = None

    def callback(self, data):
        try:
            # check if successfully subscribed the topic
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
			print(e)
			sys.exit()

        # Image processing, medianblur works well for the old camera
        median = cv2.medianBlur(cv_image, 7)
        imgGray = cv2.cvtColor(median, cv2.COLOR_BGR2GRAY)

        # Use Canny edge detection and close the contour
        edges = cv2.Canny(imgGray, 100, 200)
        kernel = np.ones((3, 3), np.uint8)
        closing = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
        dilation = cv2.dilate(closing, kernel, iterations=1)
        opening = cv2.morphologyEx(dilation, cv2.MORPH_OPEN, kernel)
        erosion = cv2.erode(opening, kernel, iterations=1)

        # find contours
        ret, threshCr = cv2.threshold(edgeCr, 127, 255, 0, cv2.THRESH_BINARY)
        ret, threshCir = cv2.threshold(edgeCir, 127, 255, 0, cv2.THRESH_BINARY)
        ret, threshCam = cv2.threshold(erosion, 127, 255, 0, cv2.THRESH_BINARY)

        _, contours, _ = cv2.findContours(threshCir, 2, 1)
        cntCir = contours[0]

        _, contours, _ = cv2.findContours(threshCr, 2, 1)
        cntCr = contours[0]

        _, contours, _ = cv2.findContours(threshCam, 2, 1)  # method, mode

        # matchshapes
        screencnt = None
        for cnt in contours:
            retCir = cv2.matchShapes(cntCir, cnt, 3, 0.0)
            retCr = cv2.matchShapes(cntCr, cnt, 3, 0.0)
            minnumb = min(retCir, retCr)

            approx = cv2.approxPolyDP(
                cnt, 0.1 * cv2.arcLength(cnt, True), True)
            area = cv2.contourArea(approx)

            if area >= 600:  # filtering out the noise by contour area

                if minnumb >= 0.02:  # filtering out the noise by cv2.matchshapes result
                    continue
                else:
                    if minnumb == retCr:  # cv2.matchshapes sometimes confuses cross and triangle, here I am using another method to differentiate these two shapes
                        if len(approx) == 3:
                            shape = "triangle"

                        else:
                            shape = "cross"
                        screencnt = cnt
                        # mask the contour
                        mask = np.zeros(imgGray.shape, np.uint8)
                        cv2.drawContours(mask, [screencnt], -1, 255, -1)
                        pixelpoints = np.transpose(np.nonzero(mask))
                        # color detection
                        mean = cv2.mean(cv_image, mask=mask)
                        blue = mean[0]
                        green = mean[1]
                        red = mean[2]
                        dominantcolor = max(blue, green, red)
                        if dominantcolor == blue:
                            color = "blue"
                        elif dominantcolor == green:
                            color = "green"
                        elif dominantcolor == red:
                            color = "red"

                        break

                    elif minnumb == retCir:
                        shape = "circle"
                        screencnt = cnt
                        mask = np.zeros(imgGray.shape, np.uint8)
                        cv2.drawContours(mask, [screencnt], -1, 255, -1)
                        pixelpoints = np.transpose(np.nonzero(mask))
                        # color detection
                        mean = cv2.mean(cv_image, mask=mask)
                        blue = mean[0]
                        green = mean[1]
                        red = mean[2]
                        dominantcolor = max(blue, green, red)
                        if dominantcolor == blue:
                            color = "blue"
                        elif dominantcolor == green:
                            color = "green"
                        elif dominantcolor == red:
                            color = "red"

                        break

        # publish the message out
        rospy.loginfo(shape)
        self.pub.publish(shape)
        rospy.loginfo(color)
        self.pub.publish(color)


def main(args):
	rospy.init_node('dock_detection', anonymous=True)
	ic = dock_detection()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
