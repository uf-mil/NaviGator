#!/usr/bin/env python
from mil_vision_tools import GaussianColorClassifier
from rospkg import RosPack
import os


class TotemsColorClassifier(GaussianColorClassifier):
    CLASSES = ['{}_totem'.format(color) for color in ['white', 'red', 'green', 'blue', 'yellow']]

    def __init__(self):
        rospack = RosPack()
        path = rospack.get_path('navigator_vision')
        training_file = os.path.join(path, 'config/totems_color/training.csv')
        labelfile = os.path.join(path, 'config/totems_color/labels.json')
        super(TotemsColorClassifier, self).__init__(self.CLASSES,
                                                    training_file=training_file, labelfile=labelfile)


if __name__ == '__main__':
    '''
    Can be run as executable to extract features or check accuracy score
    '''
    import sys
    c = TotemsColorClassifier()
    c.main(sys.argv[1:])
