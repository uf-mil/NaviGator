#!/usr/bin/env python
from mil_vision_tools import GaussianColorClassifier
from rospkg import RosPack
import os


class ScanTheCodeClassifier(GaussianColorClassifier):
    CLASSES = ['stc_off', 'stc_red', 'stc_green', 'stc_blue', 'stc_yellow']

    def __init__(self):
        rospack = RosPack()
        path = rospack.get_path('navigator_vision')
        self.features_file = os.path.join(path, 'config/stc_colors.csv')
        super(ScanTheCodeClassifier, self).__init__(ScanTheCodeClassifier.CLASSES)

    def train_from_csv(self):
        return super(ScanTheCodeClassifier, self).train_from_csv(self.features_file)

    def save_csv(self, features, classes):
        return super(ScanTheCodeClassifier, self).save_csv(features, classes, filename=self.features_file)


if __name__ == '__main__':
    '''
    When run as an executable, saves the training features to a csv file
    2 arguemnts: labelbox.io labelfile, and image directory
    '''
    import sys
    labelfile = sys.argv[1]
    image_dir = sys.argv[2]
    s = ScanTheCodeClassifier()
    features, classes = s.extract_labels(labelfile, image_dir)
    s.save_csv(features, classes)
