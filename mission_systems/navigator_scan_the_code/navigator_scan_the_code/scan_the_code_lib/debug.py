"""Shows images for debugging purposes."""
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import sys
___author___ = "Tess Bianchi"


class Debug(object):

    """Class that contains methods that assist with debugging with images."""

    def __init__(self, nh=None, w=1000, h=800, total=8, win_name="debug", wait=True):
        """
        Initialize the Debug class.

        @param w = The width of the image that smaller images are added to
        @param h = The height of the image that smaller images are added to
        @param win_name = the name of the window that is shown in opencv
        @param wait = whether or not to wait after showing the image
        """
        self.width = w
        self.height = h
        self.img = np.zeros((h, w, 3), np.uint8)
        self.total = total
        self.hor_num = total / 2
        self.vert_num = 2
        self.max_width = w / self.hor_num
        self.max_height = h / self.vert_num
        self.wait = wait
        self.nh = nh

        self.curr_w = 0
        self.curr_h = 0
        self.num_imgs = 0
        self.win_name = win_name
        self.name_to_starting = {}
        self.bridge = CvBridge()
        if nh is not None:
            self.base_topic = "/debug/scan_the_code/"
            self.topic_to_pub = {}
            self.pub = nh.advertise("/debug/scan_the_code/image", Image)

    def add_image(self, img, name, wait=33, topic="image"):
        """
        Add an image to show to either with a topic or using cv2.imshow.

        @param name = a unique key name for the image,
        use the same name if you want to switch out this image for another
        @param wait = the amount of wait time for the imshow image
        """
        if topic != "image":
            self._add_new_topic(img, name, wait, topic)
            return
        if self.wait:
            wait = 0
        if len(img.shape) == 2:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        h, w, r = img.shape
        if w > h:
            img = cv2.resize(img, (self.max_width, h * self.max_width / w))

        if h > w:
            img = cv2.resize(img, (w * self.max_height / h, self.max_height))
        h, w, r = img.shape
        if name not in self.name_to_starting:
            if self.num_imgs == self.total:
                print "Too many images"
                return
            self.name_to_starting[name] = (self.curr_w, self.curr_h)
            self.num_imgs += 1

            self.curr_w += w
            if self.num_imgs == self.total / 2:
                self.curr_w = 0
                self.curr_h = self.max_height
            if self.num_imgs > self.total / 2:
                self.name_to_starting[name] = (self.curr_w, self.curr_h)
        my_w, my_h = self.name_to_starting[name]
        self.img[my_h: my_h + h, my_w: my_w + w] = img
        if self.nh is None:
            cv2.imshow("img", self.img)
            if cv2.waitKey(wait) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
                sys.exit()

        else:
            self.pub.publish(self.bridge.cv2_to_imgmsg(self.img, "bgr8"))

    def _add_new_topic(self, img, name, wait, topic):
        pub = None
        if topic in self.topic_to_pub.keys():
            pub = self.topic_to_pub[topic]
        else:
            pub = self.nh.advertise("/debug/scan_the_code/" + topic, Image)
            self.topic_to_pub[topic] = pub
        pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
