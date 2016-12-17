"""Finds a rectangle in the image."""
import cv2
import numpy as np
import sys
import numpy.ma as ma
import numpy.linalg as npl
from navigator_tools import BagCrawler
___author___ = "Tess Bianchi"


class RectangleFinderClustering(object):
    """Class that contains functionality to find rectangles."""

    def __init__(self):
        """Initialize RectangleFinder class."""
        # PARAMS HERE
        self.change_thresh = 30

        self.prev_kmeans = None
        self.current_center = None
        self.seen_black = False
        self.color_finder = None  # ColorFinder()
        self.colors = []

    def _in_range(self, pic, x, y):
        if len(pic.shape) == 3:
            h, w, r = pic.shape
        else:
            h, w = pic.shape
        if x < 0 or x >= w:
            return False
        if y < 0 or y >= h:
            return False
        return True

    def _make_rec(self, xmin, ymin, xmax, ymax):
        return [(xmin, ymin), (xmax, ymin), (xmin, ymax), (xmax, ymax)]

    def _draw_circle(self, img, xmin, ymin, xmax, ymax, color=(255, 255, 0)):
        cv2.circle(img, (xmin, ymin), 2, color, -1)
        cv2.circle(img, (xmax, ymin), 2, color, -1)
        cv2.circle(img, (xmin, ymax), 2, color, -1)
        cv2.circle(img, (xmax, ymax), 2, color, -1)
        self.debug.add_image(img, "good", topic="points_and")

    def _has_change_in_color(self, kmeans):
        if self.prev_kmeans is None:
            return False, None
        print '2'
        max_val = None
        max_diff = -sys.maxint
        for k in kmeans:
            for p in self.prev_kmeans:
                if npl.norm(k - p) > max_diff:
                    max_diff = npl.norm(k - p)
                    max_val = k
        print "max diff", max_diff, max_val
        print kmeans, max_val
        print "val", np.where(kmeans == max_val)[0][0]

        if max_diff > 400:
            return True, np.where(kmeans == max_val)[0][0]

        return False, None

    def _get_closest_to_current(self, kmeans):
        min_val = None
        min_diff = sys.maxint
        for k in kmeans:
            if npl.norm(k - self.current_center) < min_diff:
                min_diff = npl.norm(k - self.current_center)
                min_val = k

        return np.where(kmeans == min_val)[0][0]

    def get_rectangle(self, roi, debug=None):
        """
        Get the rectangle that has changing colors in the roi.

        Returns boolean success value and the four rectangle points in the image
        """

        # Cluster the image
        draw = roi.copy()
        gaussian = cv2.GaussianBlur(roi, (9, 9), 10.0)
        roi = cv2.addWeighted(roi, 1.5, gaussian, -0.5, 0, roi)

        nh, nw, r = roi.shape

        # cluster
        Z = roi.reshape((-1, 3))
        Z = np.float32(Z)
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
        K = 6
        ret, label, centers = cv2.kmeans(Z, K, criteria, 10, 0)
        centers = np.uint8(centers)
        image_as_centers = centers[label.flatten()]
        image_as_centers = image_as_centers.reshape((roi.shape))
        labels = label.reshape((roi.shape[:2]))

        debug.add_image(image_as_centers, "labels", topic="labels")

        # Look for a big jump in the kmeans
        change, which_center = self._has_change_in_color(centers)

        print change, which_center

        if change:
            self.current_center = centers[which_center]
            # Get the centroid of that cluster
            ind = np.array(np.where(labels == which_center)).T
            centroid = np.mean(ind, axis=0)

            mask = ma.masked_equal(labels, which_center)
            mask = mask.mask.astype(np.uint8)
            mask = np.resize(mask, (mask.shape[0] + 2, mask.shape[1] + 2))
            cv2.floodFill(image_as_centers, None, tuple(centroid.astype(int)), (0, 255, 0))
            cv2.rectangle(image_as_centers, (0, 0), (30, 30), (0, 255, 0))
            debug.add_image(image_as_centers, "flood_fill", topic="flood_fill")
            # flood fill the image at that point
            # colors = ma.array(roi, mask=mask)
            # colors = colors[colors.mask].data
            # prev_color = self.color_finder.start_new_color_estimate(colors.flatten())
            # if prev_color == 'k':
            #     self.seen_black = True
            #     self.colors = []
            # elif prev_color is not None and self.seen_black:
            #     self.colors.append(prev_color)
        elif self.current_center is not None:
            which_center = self._get_closest_to_current(centers)
            mask = None
            colors = ma.array(roi, mask=mask)
            colors = colors[colors.mask].data
            self.color_finder.add_to_estimate(colors.flatten())

        self.prev_kmeans = centers
        return False, None

        # Save the positional centers of all the
        # frame = cv2.resize(roi, 0, fx=.2, fy=.2)
