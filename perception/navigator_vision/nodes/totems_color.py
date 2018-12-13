#!/usr/bin/env python
import rospy
from navigator_vision import TotemsColorClassifier
from mil_msgs.srv import ObjectDBQuery, ObjectDBQueryRequest
from mil_ros_tools import Image_Subscriber, Image_Publisher, rosmsg_to_numpy
from mil_vision_tools import roi_enclosing_points, rect_from_roi
import numpy as np
import tf
import cv2


class Totem(object):
    '''
    Keeps track of one totem's cumulative probabilities
    '''
    def __init__(self):
        self.net_probabilities = np.zeros(len(TotemsColorClassifier.CLASSES), dtype=np.float64)

    def add_sample(self, probabilities):
        self.net_probabilities += probabilities

    def classify(self):
        return np.argmax(self.net_probabilities)


class TotemsColorClassifierNode(object):
    def __init__(self):
        self.enabled = False
        self.db_proxy = rospy.ServiceProxy('/database/requests', ObjectDBQuery)
        self.debug_pub = Image_Publisher('~debug_image')
        self.image_sub = Image_Subscriber('/camera/seecam/image_rect_color', callback=self.img_cb)
        self.camera_model = self.image_sub.wait_for_camera_model()
        self.totems = {}  # Will map totem ID to
        self.objects = {}
        self.classifier = TotemsColorClassifier()
        self.classifier.train_from_csv()
        self.tf_listener = tf.TransformListener()
        self.enabled = True
        rospy.Timer(rospy.Duration(1.0), self.update_objects)

    def update_objects(self, timer):
        try:
            objects = self.db_proxy(ObjectDBQueryRequest(name='totem'))
        except rospy.ServiceException as e:
            rospy.logwarn('Error requesting totem from database: {}'.format(e))
            return
        if not objects.found:
            rospy.logwarn('No totem objects found')
            return
        for totem in objects.objects:
            self.objects[totem.id] = rosmsg_to_numpy(totem.points)

    @staticmethod
    def _errode_roi(rect):
        centerx = (rect[0][0] + rect[1][0]) / 2
        centery = (rect[0][1] + rect[1][1]) / 2
        width = 10
        height = 20
        return ((centerx - width, centery - height), (centerx + width, centery + height))  # noqa

    def img_cb(self, img):
        if not self.enabled:
            return False
        try:
            self.tf_listener.waitForTransform(self.camera_model.tfFrame(), 'enu', self.image_sub.last_image_time,
                                              rospy.Duration(1.0))
            (trans, rot) = self.tf_listener.lookupTransform(self.camera_model.tfFrame(), 'enu',
                                                            self.image_sub.last_image_time)
        except tf.Exception as e:
            rospy.logwarn('TF error: {}'.format(e))
            return
        P = np.array(trans)
        R = tf.transformations.quaternion_matrix(rot)[:3, :3]
        debug = np.zeros_like(img)
        for idx in self.objects:
            mask = np.zeros((img.shape[0], img.shape[1]), dtype=img.dtype)
            points_transformed = P + (R.dot(self.objects[idx].T)).T
            roi = roi_enclosing_points(self.camera_model, points_transformed)
            if roi is None:
                rospy.logwarn('No points for totem {}'.format(idx))
                continue
            rect = rect_from_roi(roi)
            rect = ((rect[0][0], rect[0][1] + 50), (rect[0][0] + 10, rect[1][1]))  # Hard coded for testing
            cv2.rectangle(mask, rect[0], rect[1], (255, 255, 255), -1)
            probabilities = self.classifier.probabilities(img, mask)
            if idx not in self.totems:
                self.totems[idx] = Totem()
            self.totems[idx].add_sample(probabilities[0])
            debug = cv2.bitwise_or(img, debug, mask=mask)
        for idx in self.totems:
            classification = self.totems[idx].classify()
            print '{} is {}'.format(idx, self.classifier.class_to_string(classification))
        self.debug_pub.publish(debug)


if __name__ == '__main__':
    rospy.init_node('totem_color_classifier')
    TotemsColorClassifierNode()
    rospy.spin()
