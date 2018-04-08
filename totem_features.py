#!/usr/bin/env python
from mil_vision_tools import GaussianColorClassifier


if __name__ == '__main__':
    colors = ['white', 'red', 'green', 'blue', 'yellow']
    classes = ['{}_totem'.format(c) for c in colors]
    classifer = GaussianColorClassifier(classes)
    labelfile = 'perception/navigator_vision/navigator_vision/totems_color_labels.json'
    img_dir = '/media/kallen/storage/bags/totems_color/'
    csv = 'totems_training.csv'
    features, classes = classifer.extract_labels(labelfile, img_dir)
    classifer.save_csv(features, classes, filename=csv)
    classifer.train_from_csv(csv)
    print 'Score: ', classifer.score(features, classes)

