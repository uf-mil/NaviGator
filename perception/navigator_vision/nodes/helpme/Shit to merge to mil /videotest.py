""" A class for testing a SSD model on a video file or webcam """
from __future__ import print_function

import cv2
import keras
from keras.applications.imagenet_utils import preprocess_input
from keras.backend.tensorflow_backend import set_session
from keras.models import Model
from keras.preprocessing import image
import pickle
import numpy as np
from random import shuffle
from scipy.misc import imread, imresize
from timeit import default_timer as timer
import tensorflow as tf

import sys
sys.path.append("..")
from ssd_utils import BBoxUtility

import roslib
# roslib.load_manifest('my_package')
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# global graph


class image_converter:

    def __init__(self, class_names, model, input_shape):
        self.class_names = class_names
        self.num_classes = len(class_names)
        self.model = model
        self.graph = tf.get_default_graph()
        self.input_shape = input_shape
        self.bbox_util = BBoxUtility(self.num_classes)

        # Create unique and somewhat visually distinguishable bright
        # colors for the different classes.
        self.class_colors = []
        for i in range(0, self.num_classes):
            # This can probably be written in a more elegant manner
            hue = 255 * i / self.num_classes
            col = np.zeros((1, 1, 3)).astype("uint8")
            col[0][0][0] = hue
            col[0][0][1] = 128  # Saturation
            col[0][0][2] = 255  # Value
            cvcol = cv2.cvtColor(col, cv2.COLOR_HSV2BGR)
            col = (int(cvcol[0][0][0]), int(
                cvcol[0][0][1]), int(cvcol[0][0][2]))
            self.class_colors.append(col)

    def get_predictions(self, data):

        class_names = ["Background", "Buoy", "Blue Totem",
                       "Yellow Totem", "White Totem", "Red Totem", "Green Totem", "Buoy"]
        retval = data
        orig_image = data
        conf_thresh = 0.23

        im_size = (self.input_shape[0], self.input_shape[1])
        print(im_size)
        print(orig_image.shape)
        resized = cv2.resize(orig_image, im_size)
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)

        (rows, cols, channels) = data.shape
        vidw = cols
        vidh = rows
        vidar = vidw/vidh
        start_frame = 0
        video_path = 0

        # Reshape to original aspect ratio for later visualization
        # The resized version is used, to visualize what kind of resolution
        # the network has to work with.
        to_draw = cv2.resize(
            resized, (int(self.input_shape[0] * vidar), self.input_shape[1]))

        # Use model to predict
        inputs = [image.img_to_array(rgb)]
        tmp_inp = np.array(inputs)
        x = preprocess_input(tmp_inp)
        # global graph
        with self.graph.as_default():
            y = self.model.predict(x)

        # This line creates a new TensorFlow device every time. Is there a
        # way to avoid that?
        results = self.bbox_util.detection_out(y)
        print(results)
        if len(results) > 0 and len(results[0]) > 0:
            # Interpret output, only one frame is used
            det_label = results[0][:, 0]
            det_conf = results[0][:, 1]
            det_xmin = results[0][:, 2]
            det_ymin = results[0][:, 3]
            det_xmax = results[0][:, 4]
            det_ymax = results[0][:, 5]

            top_indices = [i for i, conf in enumerate(
                det_conf) if conf >= conf_thresh]

            top_conf = det_conf[top_indices]
            top_label_indices = det_label[top_indices].tolist()
            top_xmin = det_xmin[top_indices]
            top_ymin = det_ymin[top_indices]
            top_xmax = det_xmax[top_indices]
            top_ymax = det_ymax[top_indices]
            exports = []
            labels = []
            for i in range(top_conf.shape[0]):
                xmin = int(round(top_xmin[i]))
                ymin = int(round(top_ymin[i]))
                xmax = int(round(top_xmax[i]))
                ymax = int(round(top_ymax[i]))

                yret = (ymin + ymax) / 2
                xret = (xmin + xmax) / 2
                # Draw the box on top of the to_draw image
                class_num = int(top_label_indices[i])

                class_name = class_names[class_num]
                ex = np.array([xret, yret])
                exports.append(ex)
                labels.append(class_name)

        # cv2.imshow("SSD result", to_draw)
        # cv2.waitKey(10)
        # print(to_draw)

        return exports, labels
