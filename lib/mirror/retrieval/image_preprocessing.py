#!/usr/bin/env python
"""
Some image pre-processing general functions
"""
from __future__ import print_function

import tensorflow as tf

def random_sample_and_crop(img_buffer, channel, aspect_change, min_object_coverage, seed=0):
    # sample a bounding box
    bbox = tf.constant([0.0, 0.0, 1.0, 1.0], dtype=tf.float32, shape=[1, 1, 4])
    bbox_begin, bbox_size, bbox_for_draw = tf.image.sample_distorted_bounding_box(
        tf.image.extract_jpeg_shape(img_buffer),
        seed=seed,
        bounding_boxes=bbox,
        min_object_covered=min_object_coverage,
        aspect_ratio_range=[aspect_change, 1.0 / aspect_change],
        use_image_if_no_bounding_boxes=True,
        name='sample_image_bounding_box')

    offset_y, offset_x, _ = tf.unstack(bbox_begin)
    target_height, target_width, _ = tf.unstack(bbox_size)
    crop_window = tf.stack([offset_y, offset_x, target_height, target_width])
    img = tf.image.decode_and_crop_jpeg(img_buffer, crop_window, channels=channel)
    img = tf.cast(img, tf.float32)
    return img


def center_crop_preserving_aspect(img_buffer, channel, img_size=224, enlarge_first=False, long_or_short='short'):
    img = tf.image.decode_jpeg(img_buffer, channels=channel)
    # aspect preserving resize
    shape = tf.shape(img)
    height, width = tf.cast(shape[0], tf.float32), tf.cast(shape[1], tf.float32)
    if long_or_short == 'short':
        smaller_dim = tf.minimum(height, width)
        if enlarge_first:
            # This is same as the data preprocessing used in tensorflow/resnet/imagenet example
            # resize to 256 then to 224 works better than directly resize to 224 (maybe localization?)
            resize_min = int(img_size) + 32
        else:
            # It is found that on Oxford, it is better to retain the original information.
            resize_min = int(img_size)
    elif long_or_short == 'long':
        smaller_dim = tf.maximum(height, width)
        resize_min = int(img_size)
    else:
        raise NotImplementedError()
    scale_ratio = resize_min / smaller_dim
    height = tf.cast(tf.ceil(height * scale_ratio), tf.int32)
    width = tf.cast(tf.ceil(width * scale_ratio), tf.int32)
    img = tf.image.resize_images(img, [height, width])

    if long_or_short == 'short':
        # central crop
        crop_height = img_size
        crop_width = img_size
        amount_to_be_cropped_h = (height - crop_height)
        crop_top = amount_to_be_cropped_h // 2
        amount_to_be_cropped_w = (width - crop_width)
        crop_left = amount_to_be_cropped_w // 2
        return tf.slice(img, [crop_top, crop_left, 0], [crop_height, crop_width, -1])
    elif long_or_short == 'long':
        img = tf.image.resize_image_with_crop_or_pad(img, img_size, img_size)
        return img


def data_preprocess(spec, img, add_random):
    """Data Preprocess.
    Args:
        spec: network specifications.
        img: image input (in batch).
        name: name of input preprocess module.
    Returns:
        img: preprocessed images.
    """
    def add_randomness(img):
        random_contrast = tf.image.random_contrast(img, 0.50, 1.50)
        random_saturation = tf.image.random_saturation(random_contrast, 0.50, 1.50)
        random_hue = tf.image.random_hue(random_saturation, 0.20)
        random_brightness_image = tf.image.random_brightness(random_hue, 0.20)
        random_img = tf.image.random_flip_left_right(random_brightness_image)
        return random_img

    img = tf.cond(add_random, lambda: add_randomness(img), lambda: img)
    img = tf.cast(img, tf.float32)
    img = tf.subtract(img, spec.mean)
    return img
