'''
This is my implementation, or attempted one, of a Single Shot Detector using Convolutional Neural Networks.
We shall see how this goes.

Interestingly, this network builds atop the VGG-16 network. Thus we should import this.

TODO:
Produce fixed-size collection of bounding boxes. (I guess this means only make say 20 or 30 boxes?)
Produce scores for the presence of object class instances in each box (basically we are checking to see if an object is there)
Non-maximum suppression step to produce final detections (Take the image and suppress any values not close to the local maximum of the image)
  -- I think the above also involves a gradient map not the actual image.

Traditional network is first implemented. At its end are scaling feature maps.
'''

import keras.backend as K
from keras.layers import Activation
from keras.layers import AtrousConvolution2D
from keras.layers import Convolution2D
from keras.layers import Dense
from keras.layers import Flatten
from keras.layers import GlobalAveragePooling2D
from keras.layers import Input
from keras.layers import MaxPooling2D
from keras.layers import merge
from keras.layers import Reshape
from keras.layers import ZeroPadding2D
from keras.models import Model

from layers import Normalize
from layers import PriorBox

#############################################################
# Take in a standard network as a model, in this case VGG-16#
# Follows a naming format of 'KEYNAME-POOLING_STAGE-LAYER ' #
#############################################################


def SSD300(input_shape, num_classes=21):
    net = {}

    input_tensor = input_tensor = Input(shape=input_shape)
    img_size = (input_shape[1], input_shape[0])
    net['input'] = input_tensor
    # First layer of convolution === Begin Block 1
    net['vgg-1-1'] = Convolution2D(64, 3, 3, border_mode='same',
                                   activation='relu')(net['input'])
    # Zero pad results
    # net['zero-1-2'] = ZeroPadding2D((1, 1))
    # Second layer
    net['vgg-1-2'] = Convolution2D(64, 3, 3, border_mode='same',
                                   activation='relu')(net['vgg-1-1'])
    # First pooling
    net['pooling-1'] = MaxPooling2D((2, 2), border_mode='same',
                                    strides=(2, 2))(net['vgg-1-2'])
    # net['zero-1-2'] = ZeroPadding2D((1, 1))

    # Begin Block 2
    net['vgg-2-1'] = Convolution2D(128, 3, 3, border_mode='same',
                                   activation='relu')(net['pooling-1'])
    # net['zero-2-1'] = ZeroPadding2D((1, 1))
    net['vgg-2-2'] = Convolution2D(128, 3, 3, border_mode='same',
                                   activation='relu')(net['vgg-2-1'])
    net['pooling-2'] = MaxPooling2D((2, 2), border_mode='same',
                                    strides=(2, 2))(net['vgg-2-2'])
    # net['zero-2-2'] = ZeroPadding2D((1, 1))

    # Begin Block 3
    net['vgg-3-1'] = Convolution2D(256, 3, 3, border_mode='same',
                                   activation='relu')(net['pooling-2'])
    # net['zero-3-1'] = ZeroPadding2D((1, 1))
    net['vgg-3-2'] = Convolution2D(256, 3, 3, border_mode='same',
                                   activation='relu')(net['vgg-3-1'])
    net['vgg-3-2-1'] = Convolution2D(256, 3, 3, border_mode='same',
                                     activation='relu')(net['vgg-3-2'])
    net['pooling-3'] = MaxPooling2D((2, 2), border_mode='same',
                                    strides=(2, 2))(net['vgg-3-2-1'])
    # net['zero-3-2'] = ZeroPadding2D((1, 1))

    # Begin Block 4
    net['vgg-4-1'] = Convolution2D(512, 3, 3, border_mode='same',
                                   activation='relu')(net['pooling-3'])
    # net['zero-4-1'] = ZeroPadding2D((1, 1))
    net['vgg-4-2'] = Convolution2D(512, 3, 3, border_mode='same',
                                   activation='relu')(net['vgg-4-1'])
    net['vgg-4-2-1'] = Convolution2D(512, 3, 3, border_mode='same',
                                     activation='relu')(net['vgg-4-2'])
    net['pooling-4'] = MaxPooling2D((2, 2), border_mode='same',
                                    strides=(2, 2))(net['vgg-4-2-1'])
    # net['zero-4-2'] = ZeroPadding2D((1, 1))

    # Begin Block 5
    net['vgg-5-1'] = Convolution2D(512, 3, 3, border_mode='same',
                                   activation='relu')(net['pooling-4'])
    # net['zero-5-1'] = ZeroPadding2D((1, 1))
    net['vgg-5-2'] = Convolution2D(512, 3, 3, border_mode='same',
                                   activation='relu')(net['vgg-5-1'])
    net['vgg-5-2-1'] = Convolution2D(512, 3, 3, border_mode='same',
                                     activation='relu')(net['vgg-5-2'])
    net['pooling-5'] = MaxPooling2D((3, 3), border_mode='same',
                                    strides=(1, 1))(net['vgg-5-2-1'])
    # Begin the wierd SSD stuff 6
    net['inter_stage-7-1'] = AtrousConvolution2D(
        1024, 3, 3, atrous_rate=(6, 6), border_mode='same', activation='relu')(net['pooling-5'])

    net['inter_stage-7-2'] = Convolution2D(1024, 1,
                                           1, border_mode='same', activation='relu')(net['inter_stage-7-1'])

    # Begin Block 7
    net['ssd-7-1'] = Convolution2D(256, 1, 1, border_mode='same',
                                   activation='relu')(net['inter_stage-7-2'])
    net['ssd-7-2'] = Convolution2D(512, 3, 3, border_mode='same',
                                   subsample=(2, 2), activation='relu')(net['ssd-7-1'])

    # Begin Block 8
    net['ssd-7-3'] = Convolution2D(128, 1, 1, border_mode='same',
                                   activation='relu')(net['ssd-7-2'])
    net['ssd-7-4'] = ZeroPadding2D()(net['ssd-7-3'])

    net['ssd-7-4'] = Convolution2D(256, 3, 3, border_mode='valid',
                                   subsample=(2, 2), activation='relu')(net['ssd-7-4'])
    # Begin Block 9
    net['ssd-7-5'] = Convolution2D(128, 1, 1, border_mode='same',
                                   activation='relu')(net['ssd-7-4'])
    net['ssd-7-6'] = Convolution2D(256, 3, 3,
                                   subsample=(2, 2), activation='relu', border_mode='same')(net['ssd-7-5'])

    # Final Pooling Stages
    net['pooling-7'] = GlobalAveragePooling2D(
        name='pooling-7')(net['ssd-7-6'])

    ##########################################################################
    # Prediction and bounding box creation stage. This is where we will be   #
    # attempting to classify an object based on different stages of this     #
    # network. The idea behind this is by accessing the results of different #
    # layers we will be able to classify the object at different sizes.      #
    # This is because each layer resizes the image.                          #
    ##########################################################################

    # Prediction Stages - Starting with Block 4 predictions
    net['norm-1'] = Normalize(20, name='norm-1')(net['vgg-4-2-1'])

    num_priors = 3

    x = Convolution2D(num_priors * 4,
                      3, 3, border_mode='same')(net['norm-1'])
    net['norm_mbox_loc-1'] = x
    zed = Flatten(name='norm_mbox_loc-1')
    net['norm_mbox_flat-1'] = zed(net['norm_mbox_loc-1'])
    name = 'norm_mbox_conf_bef-1'

    if num_classes != 21:
        name += '_{}'.format(num_classes)

    x = Convolution2D(num_priors * num_classes, 3, 3,
                      border_mode='same')(net['norm-1'])
    net['norm_mbox_conf_bef-1'] = x
    zed = Flatten(name='norm_mbox_conf-1')
    net['norm_mbox_conf-1'] = zed(net['norm_mbox_conf_bef-1'])
    priorbox = PriorBox(img_size, 30.0, aspect_ratios=[2], variances=[
        0.1, 0.1, 0.2, 0.2], name='norm_mbox_prior-1')

    net['norm_mbox_prior-1'] = priorbox(net['norm-1'])

    # Block inter-stage Predictions

    # net['norm-2'] = Normalize(20, name='norm-2')(net['inter_stage-7-2'])

    num_priors = 6

    net['norm_mbox_loc-2'] = Convolution2D(num_priors * 4,
                                           3, 3,  border_mode='same')(net['inter_stage-7-2'])
    zed = Flatten(name='norm_mbox_loc-2')
    net['norm_mbox_flat-2'] = zed(net['norm_mbox_loc-2'])
    name = 'norm_mbox_conf_bef-2'

    if num_classes != 21:
        name += '_{}'.format(num_classes)

    net['norm_mbox_conf_bef-2'] = Convolution2D(num_priors * num_classes, 3, 3,
                                                border_mode='same')(net['inter_stage-7-2'])
    zed = Flatten(name='norm_mbox_conf-2')
    net['norm_mbox_conf-2'] = zed(net['norm_mbox_conf_bef-2'])
    priorbox = PriorBox(img_size, 60.0, max_size=114.0, aspect_ratios=[2, 3], variances=[
        0.1, 0.1, 0.2, 0.2], name='norm_mbox_prior-2')

    net['norm_mbox_prior-2'] = priorbox(net['inter_stage-7-2'])

    # Block 7 Predictions

    # net['norm-3'] = Normalize(20, name='norm-3')(net['ssd-7-2'])

    num_priors = 6

    x = Convolution2D(num_priors * 4,
                      3, 3, border_mode='same')(net['ssd-7-2'])
    net['norm_mbox_loc-3'] = x
    zed = Flatten(name='norm_mbox_loc-3')
    net['norm_mbox_flat-3'] = zed(net['norm_mbox_loc-3'])
    name = 'norm_mbox_conf_bef-3'

    if num_classes != 21:
        name += '_{}'.format(num_classes)

    x = Convolution2D(num_priors * num_classes, 3, 3,
                      border_mode='same')(net['ssd-7-2'])
    net['norm_mbox_conf_bef-3'] = x
    zed = Flatten(name='norm_mbox_conf-3')
    net['norm_mbox_conf-3'] = zed(net['norm_mbox_conf_bef-3'])
    priorbox = PriorBox(img_size, 114.0, max_size=168.0, aspect_ratios=[2, 3], variances=[
        0.1, 0.1, 0.2, 0.2], name='norm_mbox_prior-3')

    net['norm_mbox_prior-3'] = priorbox(net['ssd-7-2'])

    # Block 8 predictions
    # net['norm-4'] = Normalize(20, name='norm-4')(net['ssd-7-4'])

    num_priors = 6

    x = Convolution2D(num_priors * 4,
                      3, 3, border_mode='same')(net['ssd-7-4'])
    net['norm_mbox_loc-4'] = x
    zed = Flatten(name='norm_mbox_loc-4')
    net['norm_mbox_flat-4'] = zed(net['norm_mbox_loc-4'])
    name = 'norm_mbox_conf_bef-4'

    if num_classes != 21:
        name += '_{}'.format(num_classes)

    x = Convolution2D(num_priors * num_classes, 3, 3,
                      border_mode='same')(net['ssd-7-4'])
    net['norm_mbox_conf_bef-4'] = x
    zed = Flatten(name='norm_mbox_conf-4')
    net['norm_mbox_conf-4'] = zed(net['norm_mbox_conf_bef-4'])

    priorbox = PriorBox(img_size, 168.0, max_size=222.0, aspect_ratios=[2, 3], variances=[
        0.1, 0.1, 0.2, 0.2])

    net['norm_mbox_prior-4'] = priorbox(net['ssd-7-4'])

    # Block 9 Predictions
    # net['norm-5'] = Normalize(20, name='norm-5')(net['ssd-7-6'])

    num_priors = 6

    x = Convolution2D(num_priors * 4,
                      3, 3, border_mode='same')(net['ssd-7-6'])
    net['norm_mbox_loc-5'] = x
    zed = Flatten(name='norm_mbox_loc-5')
    net['norm_mbox_flat-5'] = zed(net['norm_mbox_loc-5'])
    name = 'norm_mbox_conf_bef-5'
    if num_classes != 21:
        name += '_{}'.format(num_classes)

    x = Convolution2D(num_priors * num_classes, 3, 3,
                      border_mode='same')(net['ssd-7-6'])
    net['norm_mbox_conf_bef-5'] = x
    zed = Flatten(name='norm_mbox_conf-5')
    net['norm_mbox_conf-5'] = zed(net['norm_mbox_conf_bef-5'])

    priorbox = PriorBox(img_size, 222.0, max_size=276.0, aspect_ratios=[2, 3], variances=[
        0.1, 0.1, 0.2, 0.2], name='norm_mbox_prior-5')

    net['norm_mbox_prior-5'] = priorbox(net['ssd-7-6'])

    # Final Pooling Stage Predictions
    num_priors = 6

    x = Dense(num_priors * 4, name='norm_mbox_flat-6')(net['pooling-7'])
    net['norm_mbox_flat-6'] = x

    name = 'norm_mbox_conf_bef-6'
    if num_classes != 21:
        name += '_{}'.format(num_classes)

    x = Dense(num_priors *
              num_classes, name=name)(net['pooling-7'])
    net['norm_mbox_conf-6'] = x

    priorbox = PriorBox(img_size, 276.0, max_size=330.0, aspect_ratios=[2, 3], variances=[
        0.1, 0.1, 0.2, 0.2], name='norm_mbox_prior-6')

    # net['norm_mbox_prior-6'] = priorbox(net['norm-6'])
    if K.image_dim_ordering() == 'tf':
        target_shape = (1, 1, 256)
    else:
        target_shape = (256, 1, 1)
    net['pool7_reshaped'] = Reshape(
        target_shape, name='pool7_reshaped')(net['pooling-7'])
    net['pool7_mbox_priorbox'] = priorbox(net['pool7_reshaped'])

    ##########################################################################
    # Gather all predictions to form final classification/bounding boxes.    #
    # Returns a model.                                                       #
    ##########################################################################

    # for i in range(1, 7):
    #     print(i, "Loc_Flat", net['norm_mbox_flat-{0}'.format(i)]._keras_shape)
    #     print(i, "Conf_Flat", net['norm_mbox_conf-{0}'.format(i)]._keras_shape)
    #     if i < 5:
    #         print(i, "Prior_Flat", net[
    #               'norm_mbox_prior-{0}'.format(i)]._keras_shape)
    #     else:
    #         print(i, "Prior_Flat", net['pool7_mbox_priorbox']._keras_shape)

    net['mbox_loc'] = merge([net['norm_mbox_flat-1'],
                             net['norm_mbox_flat-2'],
                             net['norm_mbox_flat-3'],
                             net['norm_mbox_flat-4'],
                             net['norm_mbox_flat-5'],
                             net['norm_mbox_flat-6']],
                            mode='concat', concat_axis=1, name='mbox_loc')
    net['mbox_conf'] = merge([net['norm_mbox_conf-1'],
                              net['norm_mbox_conf-2'],
                              net['norm_mbox_conf-3'],
                              net['norm_mbox_conf-4'],
                              net['norm_mbox_conf-5'],
                              net['norm_mbox_conf-6']],
                             mode='concat', concat_axis=1, name='mbox_conf')
    net['mbox_priorbox'] = merge([net['norm_mbox_prior-1'],
                                  net['norm_mbox_prior-2'],
                                  net['norm_mbox_prior-3'],
                                  net['norm_mbox_prior-4'],
                                  net['norm_mbox_prior-5'],
                                  net['pool7_mbox_priorbox']],
                                 mode='concat', concat_axis=1,
                                 name='mbox_priorbox')
    if hasattr(net['mbox_loc'], '_keras_shape'):
        num_boxes = net['mbox_loc']._keras_shape[-1] // 4
    elif hasattr(net['mbox_loc'], 'int_shape'):
        num_boxes = K.int_shape(net['mbox_loc'])[-1] // 4
    net['mbox_loc'] = Reshape((num_boxes, 4),
                              name='mbox_loc_final')(net['mbox_loc'])
    net['mbox_conf'] = Reshape((num_boxes, num_classes),
                               name='mbox_conf_logits')(net['mbox_conf'])
    net['mbox_conf'] = Activation('softmax',
                                  name='mbox_conf_final')(net['mbox_conf'])
    net['predictions'] = merge([net['mbox_loc'],
                                net['mbox_conf'],
                                net['mbox_priorbox']],
                               mode='concat', concat_axis=2,
                               name='predictions')
    model = Model(net['input'], net['predictions'])
    return model
