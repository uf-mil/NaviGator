#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from mil_ros_tools import Image_Subscriber, Image_Publisher, rosmsg_to_numpy
from mil_vision_tools import auto_canny, RectFinder, ImageMux, contour_mask, putText_ul, roi_enclosing_points
from collections import deque
from navigator_vision import ScanTheCodeClassifier
from mil_msgs.srv import ObjectDBQuery, ObjectDBQueryRequest
import tf
from image_geometry import PinholeCameraModel


class ScanTheCodePerception(object):
    LED_HEIGHT = 0.38608
    LED_WIDTH = 0.19304

    def __init__(self):
        self.enabled = False
        self.tf_listener = tf.TransformListener()
        self.roi = None
        self.img = None
        self.get_params()
        self.rect_finder = RectFinder(self.LED_HEIGHT, self.LED_WIDTH)
        self.db_service = rospy.ServiceProxy('/database/requests', ObjectDBQuery)
        self.sub = Image_Subscriber(self.image_topic, self.img_cb)
        info = self.sub.wait_for_camera_info()
        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(info)
        if self.debug:
            self.debug_pub = Image_Publisher('~debug_image')
            res = 2
            self.image_mux = ImageMux(size=(info.height * res, info.width * res), shape=(2, 2),
                                      labels=['Original', 'Blur', 'Contours', 'Classification'])
        self.classification_list = deque()  # "DECK - AH - WAY - WAY"
        self.enabled = True
        rospy.Timer(rospy.Duration(0.25), self.update_roi)

    def get_params(self):
        '''
        Set several constants used for image processing and classification
        from ROS params for runtime configurability.
        '''
        self.debug = rospy.get_param('~debug', True)
        self.image_topic = rospy.get_param('~image_topic', '/camera/seecam/image_rect_color')
        self.min_contour_area = rospy.get_param('~min_contour_area', 50)
        self.filter_d = rospy.get_param('filter_d', 5)
        self.filter_sigma = rospy.get_param('filter_sigma', 50)
        self.classifier = ScanTheCodeClassifier()
        self.classifier.train_from_csv()

    def update_roi(self, timer_obj):
        '''
        Update the region of interest where the LED panel os
        '''
        if not self.enabled:  # Ignore if not enabled
            return
        # Get points from database
        try:
            res = self.db_service(ObjectDBQueryRequest(name='stc'))
        except rospy.ServiceException as e:
            rospy.logwarn('Database service error: {}'.format(e))
            return
        if not res.found:
            rospy.logwarn('Scan the code object not found')
            return
        obj = res.objects[0]

        # Get transform
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.camera_model.tfFrame(),
                                                            obj.header.frame_id, rospy.Time(0))
        except tf.Exception as e:
            rospy.logwarn('TF error betwen {} and {}: {}'.format(self.camera_model.tfFrame(), obj.header.frame_id, e))
            return
        P = np.array(trans)
        R = tf.transformations.quaternion_matrix(rot)[:3, :3]
        points = rosmsg_to_numpy(obj.points)
        points_transformed = P + (R.dot(points.T)).T
        self.roi = roi_enclosing_points(self.camera_model, points_transformed)

    def get_stc_mask(self, img):
        # gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        blur = cv2.bilateralFilter(img[self.roi], 9, 75, 75)
        edges = auto_canny(blur, sigma=0.5)

        if self.debug:
            self.image_mux[1] = blur
            self.image_mux[2] = edges
        return None
        c, contours, hierarchy = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

        filtered_contours = filter(lambda c: cv2.contourArea(c) > self.min_contour_area, contours)
        if len(filtered_contours) == 0:
            print 'none after area fiter'
            return None

        matches = map(self.rect_finder.verify_contour, filtered_contours)
        matches_sorted_index = np.argsort(matches)
        print 'best', matches[matches_sorted_index[0]]
        if matches[matches_sorted_index[0]] > 1.0:
            print 'no good rectangle'
            return None
        sorted_matches = np.array(matches)[matches_sorted_index]
        match = None
        for i in sorted_matches:
            pts = self.rect_finder.get_corners(i, epsilon_range=(0.01, 0.05))
            if pts is None:
                continue
            _, _, width, height = cv2.boundingRect(pts)
            # STC is always upright
            ratio = float(height) / width
            # print 'ratio ', ratio
            if ratio < 1 or ratio > 5:
                continue
            match = i
            break
        return match

    def img_cb(self, img):
        if not self.enabled:
            return
        if self.roi is None:
            rospy.logwarn_throttle(1.0, 'no roi')
            return
        self.image_mux[0] = img
        match = self.get_stc_mask(img)
        if match is not None:
            debug = contour_mask(match, img_shape=img.shape)
            prediction = self.classifier.classify(img, debug)
            label = self.classifier.CLASSES[prediction]
            symbol = label[0]
            if len(self.classification_list) == 0 or self.classification_list[-1] != symbol:
                if len(self.classification_list) >= 5:
                    self.classification_list.popleft()
                self.classification_list.append(symbol)
            text = label + ' | ' + ''.join(self.classification_list)
            scale = 3
            thickness = 2
            (text_width, text_height), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_COMPLEX_SMALL, scale, thickness)
            putText_ul(debug, text, fontScale=scale, thicknesss=thickness)
            debug = cv2.bitwise_or(img, img, mask=debug)
            if self.debug:
                self.image_mux[3] = debug
        if self.debug:
            self.debug_pub.publish(self.image_mux())


if __name__ == '__main__':
    rospy.init_node('scan_the_code_perception')
    s = ScanTheCodePerception()
    rospy.spin()
