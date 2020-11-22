#!/usr/bin/env python
"""
Copyright 2017, Tianwei Shen, HKUST.
Copyright 2017, Zixin Luo, HKUST.
Model architectures.
"""
from __future__ import print_function

import sys
import os
import numpy as np

import tensorflow as tf
from tensorflow.python.ops import gen_nn_ops
from scipy import ndimage

sys.path.append("../")
from cnn_wrapper import helper, GoogleNet, GoogleNet_Places, GoogleNetBN, Half_GoogleNet, \
    ResNet50, ResNet101, ResNet152
from tools.common import Notify, read_list
from image_preprocessing import data_preprocess, center_crop_preserving_aspect, random_sample_and_crop


FLAGS = tf.app.flags.FLAGS


def recover_from_pretrained(sess, net, ckpt_path=None, ckpt_step=-1, np_model_path=None):
    """
    recover from pretrained ckpt graph or numpy net parameters
    :param sess: the current session
    :param net: the feature tower
    :param ckpt_path: the ckpt root path (if there is any)
    :param ckpt_step: the ckpt step number (if there is any)
    :param np_model_path: the numpy model path (if there is any)
    :return: the correct step index for the current
    """
    if ckpt_step is not None:       # Finetune from ckpt if necessary.
        ckpt_name = '-'.join([ckpt_path, str(ckpt_step)])
        if os.path.exists(ckpt_name + '.index'):
            restorer = tf.train.Saver(tf.global_variables())
            restorer.restore(sess, ckpt_name)
            print(Notify.INFO, 'Pre-trained model restored from', ckpt_name, Notify.ENDC)
            return ckpt_step
        else:
            print(Notify.WARNING, 'ckpt file %s does not exist, begin training from scratch' %
                  ckpt_name + '.index', Notify.ENDC)
    elif np_model_path != '':   # Finetune from a numpy file
        print(Notify.INFO, 'Recover from pre-trained model', np_model_path, Notify.ENDC)
        net.load(np_model_path, sess, ignore_missing=True)
    else:
        print(Notify.WARNING, 'Train from scratch.', Notify.ENDC)

    return 0


def get_net_func(net_type):
    if net_type == 'googlenet':
        fn = GoogleNet
    elif net_type == 'half_googlenet':
        fn = Half_GoogleNet
    elif net_type == 'googlenet_places':
        fn = GoogleNet_Places
    elif net_type == 'googlenet_bn':
        fn = GoogleNetBN
    elif net_type == 'vgg':
        fn = VGG16
    elif net_type == 'resnet-50':
        fn = ResNet50
    elif net_type == 'resnet-101':
        fn = ResNet101
    elif net_type == 'resnet-152':
        fn = ResNet152
    else:
        print(Notify.FAIL, 'No net type', net_type, Notify.ENDC)
        exit(-1)
    return fn


def multi_scale_feature(rvec, fn, image, output_tensor_name, rmac_step, reduce_method, deploy):
    # TODO: allow dynamic input size.
    if FLAGS.img_size == 896 or FLAGS.img_size == 1024:
        scale_num = 3
    elif FLAGS.img_size == 448:
        scale_num = 2
    elif FLAGS.img_size == 224:
        scale_num = 1
    else:
        raise NotImplementedError()

    if scale_num > 1:
        for i in range(scale_num - 1):
            scale_img = tf.image.resize_images(image, (FLAGS.img_size // ((i + 1) * 2), FLAGS.img_size // ((i + 1) * 2)))
            scale_net = fn({'data': scale_img}, is_training=False, reuse=True, fcn=True)
            scale_feat_map = scale_net.get_output_by_name(output_tensor_name)
            scale_rvec = _rmac(scale_feat_map, rmac_step, reduce_method, deploy)
            rvec = tf.concat([rvec, scale_rvec], axis=-1)
    return rvec


def deploy(net_type, rmac_step, output_tensor_name, reduce_method='MAX'):
    fn = get_net_func(net_type)
    spec = helper.get_data_spec(model_class=fn)
    image = tf.placeholder(tf.float32, shape=(None, FLAGS.img_size, FLAGS.img_size, spec.channels), name='input')
    net = fn({'data': image}, is_training=False, reuse=False, fcn=True)
    feat_map = net.get_output_by_name(output_tensor_name)
    rvec = _rmac(feat_map, rmac_step, reduce_method, deploy=True)
    if FLAGS.multi_scale:
        rvec = multi_scale_feature(rvec, fn, image, output_tensor_name, rmac_step, reduce_method, deploy=True)
    return net, rvec


def inference(sess, img_list, net_type, rmac_step, output_tensor_name, reduce_method='L2'):
    """Model for feature inference.
    Args:
        img_list: a string list of image path.
    Returns:
        net: net graph definition.
        batch_size: batch size.
    """
    fn = get_net_func(net_type)
    spec = helper.get_data_spec(model_class=fn)

    # make image_list length to be multiples of batch_size
    image_num = len(img_list)
    if FLAGS.img_size is not None:    # each batch has spec.batch_size images
        last_batch_add = spec.batch_size - (image_num % spec.batch_size)
        if image_num % spec.batch_size != 0:
            for i in range(last_batch_add):
                img_list.append(img_list[image_num - 1])

    batch_img, batch_size = _testing_data_queue(spec, img_list)
    data_iterator = batch_img.get_next()
    image = tf.concat(data_iterator[1], axis=0)
    image.set_shape([spec.batch_size, FLAGS.img_size, FLAGS.img_size, spec.channels])

    net = fn({'data': image}, is_training=False, reuse=False, fcn=True)
    feat_map = net.get_output_by_name(output_tensor_name)
    rvec = _rmac(feat_map, rmac_step, reduce_method, deploy=False)
    if FLAGS.multi_scale:
        rvec = multi_scale_feature(rvec, fn, image, output_tensor_name, rmac_step, reduce_method, deploy=False)
    sess.run(batch_img.initializer)
    return net, rvec, batch_size, data_iterator[0]


def feature_reduce(feat, mask=None, norm=False):
    if mask is not None:
        offset = tf.reduce_min(feat, axis=[1, 2], keep_dims=True)
        mask = tf.image.resize_bilinear(mask, (7, 7))
        feat = (feat - offset) * mask + offset

    kernel_size = [feat.get_shape()[1].value, feat.get_shape()[2].value]
    vec = tf.layers.max_pooling2d(feat, kernel_size, 1, 'VALID')
    vec = tf.squeeze(vec, axis=[1, 2])
    if norm:
        vec = tf.nn.l2_normalize(vec, dim=-1)
    return vec


def softmin_loss_layer(feat_anc, feat_easy, feat_hard, loss_weight, input_mask, name):
    """Loss layer."""
    with tf.name_scope(name):  # pylint: disable=not-context-manager
        def _structured_loss(dist_mat, batch_size):
            # positive distance
            easy_dist = tf.matrix_diag_part(dist_mat)
            stack_mat = tf.stack([easy_dist] * batch_size, axis=0)
            # remove the influence of diagnal element by subtracting log(2)
            omit = tf.matrix_diag(tf.stack([tf.log(2.)] * batch_size, axis=0))
            # column-wise loss
            diff_col = tf.clip_by_value(stack_mat - dist_mat, -10., 10.)
            tmp_acc_col = tf.count_nonzero(tf.nn.relu(diff_col), dtype=tf.float32)
            diff_col = tf.log(tf.exp(diff_col) + 1)
            diff_col = diff_col - omit
            loss_col = tf.reduce_sum(diff_col, axis=0) / (batch_size - 1.)
            # loss_col = tf.reduce_max(diff_col, axis=0)
            # row-wise loss
            diff_row = tf.clip_by_value(tf.matrix_transpose(stack_mat) - dist_mat, -10., 10.)
            tmp_acc_row = tf.count_nonzero(tf.nn.relu(diff_row), dtype=tf.float32)
            diff_row = tf.log(tf.exp(diff_row) + 1)
            diff_row = diff_row - omit
            loss_row = tf.reduce_sum(diff_row, axis=1) / (batch_size - 1)
            # loss_row = tf.reduce_max(diff_row, axis=1)
            # total loss
            ret_loss = tf.reduce_mean(loss_col + loss_row)
            # accuracy
            accuracy = 1. - (tmp_acc_col + tmp_acc_row) / batch_size / (batch_size - 1) / 2.
            return ret_loss, accuracy

        if input_mask != None:
            mask = tf.reshape(input_mask, [-1, 28, 14, 1])

        feat_anc = feature_reduce(feat_anc, norm=False) #, mask=mask[:, :14, :])
        feat_easy = feature_reduce(feat_easy, norm=False) #, mask=mask[:, 14:, :])
        feat_hard = feature_reduce(feat_hard, norm=False)
        batch_size = feat_anc.get_shape()[0].value

        # compute L2 norm
        norm_anc = tf.reshape(tf.reduce_sum(feat_anc * feat_anc, axis=1), [-1, 1])      # ^2 of anchor norm
        norm_easy = tf.reshape(tf.reduce_sum(feat_easy * feat_easy, axis=1), [-1, 1])   # ^2 of easy positive norm
        norm_hard = tf.reshape(tf.reduce_sum(feat_hard * feat_hard, axis=1), [-1, 1])   # ^2 of hard positive norm

        # distance matrix (Euclidean)
        easy_dist_mat = tf.sqrt(norm_anc - 2 * \
            tf.matmul(feat_anc, feat_easy, transpose_b=True) + tf.transpose(norm_easy) + 1e-6)
        hard_dist_mat = tf.sqrt(norm_anc - 2 * \
            tf.matmul(feat_anc, feat_hard, transpose_b=True) + tf.transpose(norm_hard) + 1e-6)

        easy_pos_loss, easy_accuracy = _structured_loss(easy_dist_mat, batch_size)
        hard_pos_loss, hard_accuracy = _structured_loss(hard_dist_mat, batch_size)

        loss = [loss_weight * easy_pos_loss]
        accuracy = [easy_accuracy]
        if FLAGS.with_weak:
            loss.append(loss_weight * hard_pos_loss)
            accuracy.append(hard_accuracy)

        if FLAGS.with_hard:
            # hard loss
            easy_dist = tf.matrix_diag_part(easy_dist_mat)
            hard_dist = tf.sqrt(tf.reduce_sum(tf.square(feat_anc - feat_hard), axis=1) + 1e-6)
            diff_hard = tf.clip_by_value(easy_dist - hard_dist, -10., 10.)
            diff_hard_log_loss = tf.log(tf.exp(diff_hard) + 1)
            hard_loss = tf.reduce_mean(diff_hard_log_loss)
            # compute hard accuracy
            tmp_acc_hard = tf.count_nonzero(tf.nn.relu(diff_hard), dtype=tf.float32)
            harder_accuracy = 1. - 1. * tmp_acc_hard / batch_size
            loss.append(loss_weight * hard_loss)
            accuracy.append(harder_accuracy)
        return loss, accuracy


def hinge_loss_layer(feat_anc, feat_easy, feat_hard, loss_weight, mask, name):
    """Loss layer."""
    with tf.name_scope(name):  # pylint: disable=not-context-manager
        def _structured_loss(dist_mat, batch_size, margin):
            # positive distance
            easy_dist = tf.matrix_diag_part(dist_mat)
            stack_mat = tf.stack([easy_dist] * batch_size, axis=0)
            # column-wise hinge loss
            diff_col = tf.maximum(margin + stack_mat - dist_mat, 0.)
            zero_diag = np.zeros(batch_size)
            diff_col = tf.matrix_set_diag(diff_col, zero_diag)
            tmp_acc_col = tf.count_nonzero(diff_col, dtype=tf.float32)
            loss_col = tf.reduce_sum(diff_col, axis=0) / (batch_size - 1.)
            #loss_col = tf.reduce_max(diff_col, axis=0)
            # row-wise loss
            diff_row = tf.maximum(margin + tf.matrix_transpose(stack_mat) - dist_mat, 0.)
            diff_row = tf.matrix_set_diag(diff_row, zero_diag)
            tmp_acc_row = tf.count_nonzero(diff_row, dtype=tf.float32)
            loss_row = tf.reduce_sum(diff_row, axis=1) / (batch_size - 1)
            #loss_row = tf.reduce_max(diff_row, axis=1)
            # total loss
            ret_loss = tf.reduce_mean(loss_col + loss_row) / 2.0
            # accuracy
            accuracy = 1. - (tmp_acc_col + tmp_acc_row) / batch_size / (batch_size - 1) / 2.
            return ret_loss, accuracy

        # mask = tf.reshape(mask, [-1, 28, 14, 1])
        feat_anc = feature_reduce(feat_anc, norm=True)    #mask=mask[:, :14, :])
        feat_easy = feature_reduce(feat_easy, norm=True)  #mask=mask[:, 14:, :])
        feat_hard = feature_reduce(feat_hard, norm=True)
        batch_size = feat_anc.get_shape()[0].value

        # compute L2 norm
        norm_anc = tf.reshape(tf.reduce_sum(feat_anc * feat_anc, axis=1), [-1, 1])      # ^2 of anchor norm
        norm_easy = tf.reshape(tf.reduce_sum(feat_easy * feat_easy, axis=1), [-1, 1])   # ^2 of easy positive norm
        norm_hard = tf.reshape(tf.reduce_sum(feat_hard * feat_hard, axis=1), [-1, 1])   # ^2 of hard positive norm

        # distance matrix (Euclidean)
        easy_dist_mat = norm_anc - 2 * \
            tf.matmul(feat_anc, feat_easy, transpose_b=True) + tf.transpose(norm_easy)
        hard_dist_mat = norm_anc - 2 * \
            tf.matmul(feat_anc, feat_hard, transpose_b=True) + tf.transpose(norm_hard)

        easy_margin = 0.5
        weak_margin = 0.5
        hard_margin = 0.5
        easy_pos_loss, easy_accuracy = _structured_loss(easy_dist_mat, batch_size, easy_margin)
        hard_pos_loss, hard_accuracy = _structured_loss(hard_dist_mat, batch_size, weak_margin)

        loss = [loss_weight * easy_pos_loss]
        accuracy = [easy_accuracy]
        if FLAGS.with_weak:
            loss.append(loss_weight * hard_pos_loss)
            accuracy.append(hard_accuracy)

        if FLAGS.with_hard:
            # hard loss
            easy_dist = tf.matrix_diag_part(easy_dist_mat)
            hard_dist = tf.reduce_sum(tf.square(feat_anc - feat_hard), axis=1)
            diff_hard = tf.maximum(hard_margin + easy_dist - hard_dist, 0.)
            #hard_loss = tf.reduce_mean(diff_hard)
            hard_loss = tf.reduce_max(diff_hard)
            # compute hard accuracy
            tmp_acc_hard = tf.count_nonzero(diff_hard, dtype=tf.float32)
            harder_accuracy = 1. - 1. * tmp_acc_hard / batch_size
            loss.append(loss_weight * hard_loss)
            accuracy.append(harder_accuracy)
        return loss, accuracy


def get_triplet_input_tensors(dataset_root, train_list, global_img_list, sess, net_type):
    """
    Get input data tensor from triplet list and global image list
    :param triplet_list: each line contains three comma-separated index
    :param global_img_list: global image paths corresponding to the triplet indexes.
    :return: three data tensors
    """
    fn = get_net_func(net_type)
    spec = helper.get_data_spec(model_class=fn)
    iterator = _training_data_queue(dataset_root, spec, train_list, global_img_list, sess)
    return iterator


def training(batch_sample, net_type, loss_layers):
    """
    Build training architecture.
    :param batch_sample: batch samples
    :param net_type: network type, e.g., half_googlenet, googlenet, vgg, etc.
    :param loss_layers: names of loss layers
    :return:
        tower_*: Feature towers
        loss_*: Loss of multiple stages.
        accuracy*: Accuracy for stages.
    """
    fn = get_net_func(net_type)
    spec = helper.get_data_spec(model_class=fn)
    input_sample = tf.concat(batch_sample[0:3], axis=0)
    input_mask = tf.cast(batch_sample[3], tf.float32)
    input_sample.set_shape([spec.batch_size * 3, spec.input_size[0],
                            spec.input_size[1], spec.channels])

    with tf.name_scope('feature_tower'):  # pylint: disable=not-context-manager
        feature_tower = fn({'data': input_sample}, is_training=True, reuse=False)

    def _create_loss_stage(feature_tower, loss_weight, mask, feat_name, stage_name):
        feat_stage = feature_tower.get_output_by_name(feat_name)
        feat_anc = tf.slice(feat_stage, [0, 0, 0, 0], [spec.batch_size, -1, -1, -1])
        feat_easy = tf.slice(feat_stage, [spec.batch_size, 0, 0, 0], [spec.batch_size, -1, -1, -1])
        feat_hard = tf.slice(feat_stage, [2 * spec.batch_size, 0,
                                          0, 0], [spec.batch_size, -1, -1, -1])

        # use hinge_loss_layer or softmin_loss_layer
        loss_stage, accuracy = softmin_loss_layer(
            feat_anc, feat_easy, feat_hard, loss_weight, mask, name=stage_name)
        return loss_stage, accuracy

    loss_stages = []
    accuracys = []
    loss_num = len(loss_layers)
    for i in range(len(loss_layers)):
        if i == loss_num - 1:
            loss_weight = 1.0
        else:
            loss_weight = 0.5
        loss_stage, accuracy = _create_loss_stage(
            feature_tower, loss_weight, input_mask, loss_layers[i], 'loss_stage' + str(i))
        loss_stages.append(loss_stage)
        accuracys.append(accuracy)

    _activation_summaries(tf.group(feature_tower.get_output()))
    return feature_tower, loss_stages, accuracys


def training_google_landmark(batch_sample, net_type, loss_layers):
    fn = get_net_func(net_type)
    spec = helper.get_data_spec(model_class=fn)
    input_sample = tf.concat(batch_sample[:3], axis=0)
    input_sample.set_shape([spec.batch_size * 3, spec.input_size[0], spec.input_size[1], spec.channels])

    with tf.name_scope('feature_tower'):  # pylint: disable=not-context-manager
        feature_tower = fn({'data': input_sample}, is_training=True, reuse=False)

    def _create_loss_stage(feature_tower, loss_weight, mask, feat_name, stage_name):
        feat_stage = feature_tower.get_output_by_name(feat_name)
        feat_anc = tf.slice(feat_stage, [0, 0, 0, 0], [spec.batch_size, -1, -1, -1])
        feat_easy = tf.slice(feat_stage, [spec.batch_size, 0, 0, 0], [spec.batch_size, -1, -1, -1])
        feat_hard = tf.slice(feat_stage, [2 * spec.batch_size, 0, 0, 0], [spec.batch_size, -1, -1, -1])

        # use hinge_loss_layer or softmin_loss_layer
        loss_stage, accuracy = hinge_loss_layer(feat_anc, feat_easy, feat_hard, loss_weight, mask, name=stage_name)
        return loss_stage, accuracy

    loss_stages = []
    accuracys = []
    loss_num = len(loss_layers)
    for i in range(len(loss_layers)):
        if i == loss_num - 1:
            loss_weight = 1.0
        else:
            loss_weight = 0.5
        loss_stage, accuracy = _create_loss_stage(
            feature_tower, loss_weight, None, loss_layers[i], 'loss_stage'+str(i))
        loss_stages.append(loss_stage)
        accuracys.append(accuracy)

    _activation_summaries(tf.group(feature_tower.get_output()))
    return feature_tower, loss_stages, accuracys


def _training_data_queue(dataset_root, spec, train_list, image_list, sess):
    import random

    def _get_one_batch(train_list, batch_size):
        rnd_set = random.sample(range(len(train_list)), batch_size)
        batch_record = []
        batch_mask = []
        for i in rnd_set:
            rnd_img = random.randint(0, len(train_list[i]) - 1)
            single_record = train_list[i][rnd_img]
            ref_idx = single_record[0]
            rnd_pos_idx = random.randint(0, len(single_record[1]) - 1)
            rnd_pos = single_record[1][rnd_pos_idx]
            rnd_neg = single_record[2][random.randint(0, len(single_record[2]) - 1)]
            rnd_mask = single_record[3][rnd_pos_idx]
            batch_record.append([ref_idx, rnd_pos, rnd_neg])
            batch_mask.append(rnd_mask)
        batch_record = np.stack(batch_record, axis=0)
        batch_mask = np.stack(batch_mask, axis=0)
        return batch_record, batch_mask

    def _parse_img(idx):
        img_path = tf.gather(image_list, idx)
        data = tf.read_file(tf.string_join([dataset_root, img_path]))
        img = tf.image.decode_image(data, channels=3)
        img.set_shape((224, 224, 3))
        img = data_preprocess(spec, img, tf.constant(True))
        return img

    idx_record = []
    mask_record = []
    num_sample = FLAGS.max_steps if FLAGS.ckpt_step is None else FLAGS.max_steps - FLAGS.ckpt_step
    num_sample += 1
    print(Notify.INFO, '# Samples', num_sample, Notify.ENDC)
    print(Notify.INFO, '# Batch size', spec.batch_size, Notify.ENDC)
    for _ in range(num_sample):
        one_batch_idx, one_batch_mask = _get_one_batch(train_list, spec.batch_size)
        idx_record.append(one_batch_idx)
        mask_record.append(one_batch_mask)

        # visualize training triplets.
        if False:
            from scipy.misc import imread
            import matplotlib.pyplot as plt
            one_batch_img = []
            one_batch_mask_img = []
            for i in range(len(one_batch_idx)):
                img0 = imread(os.path.join(dataset_root, image_list[one_batch_idx[i][0]]))
                img1 = imread(os.path.join(dataset_root, image_list[one_batch_idx[i][1]]))
                img2 = imread(os.path.join(dataset_root, image_list[one_batch_idx[i][2]]))
                mask = one_batch_mask[i]

                mask = np.reshape(mask, [28, 14])
                mask[:14, :] = ndimage.binary_fill_holes(mask[:14, :])
                mask[14:, :] = ndimage.binary_fill_holes(mask[14:, :])

                triplet_img = [img0, img1, img2]
                triplet_img = np.vstack(triplet_img)
                one_batch_img.append(triplet_img)
                one_batch_mask_img.append(np.uint8(mask))
            one_batch_img = np.hstack(one_batch_img)
            # show images
            plt.subplot(2, 1, 1)
            plt.imshow(one_batch_img)
            plt.imsave("batch_triplets.png", one_batch_img)
            # show masks
            one_batch_mask_img = np.hstack(one_batch_mask_img)
            plt.subplot(2, 1, 2)
            plt.imshow(one_batch_mask_img)
            plt.show()

    idx_record = np.vstack(idx_record)
    mask_record = np.vstack(mask_record)

    dataset_anc_ph = tf.placeholder(idx_record.dtype, idx_record[:, 0].shape)
    dataset_pos_ph = tf.placeholder(idx_record.dtype, idx_record[:, 1].shape)
    dataset_neg_ph = tf.placeholder(idx_record.dtype, idx_record[:, 2].shape)
    mask_record_ph = tf.placeholder(mask_record.dtype, mask_record.shape)

    dataset_anc = tf.data.Dataset.from_tensor_slices(dataset_anc_ph).map(_parse_img)
    dataset_pos = tf.data.Dataset.from_tensor_slices(dataset_pos_ph).map(_parse_img)
    dataset_neg = tf.data.Dataset.from_tensor_slices(dataset_neg_ph).map(_parse_img)
    dataset_mask = tf.data.Dataset.from_tensor_slices(mask_record_ph)

    all_dataset = tf.data.Dataset.zip((dataset_anc, dataset_pos, dataset_neg, dataset_mask))
    all_dataset = all_dataset.batch(spec.batch_size).prefetch(spec.batch_size*4)

    iterator = all_dataset.make_initializable_iterator()
    sess.run(iterator.initializer, feed_dict={dataset_anc_ph: idx_record[:, 0],
                                              dataset_pos_ph: idx_record[:, 1],
                                              dataset_neg_ph: idx_record[:, 2],
                                              mask_record_ph: mask_record})
    return iterator


def _testing_data_queue(spec, img_list):
    """Queue to read testing image list.
    Args:
        spec: Model specifications.
        img_list: List of images.
    Returns:
        batche_img: Mini-batch of decoded images.
    """
    def _parse_img(img_path, is_training):
        with tf.device('/cpu:0'):
            img_buffer = tf.read_file(img_path)
            img = center_crop_preserving_aspect(img_buffer, spec.channels, FLAGS.img_size)
            img = data_preprocess(spec, img, add_random=is_training)
        return img

    with tf.name_scope('data_queue'):  # pylint: disable=not-context-manager
        img_dataset = tf.data.Dataset.from_tensor_slices(img_list).map(
            lambda img_path: _parse_img(img_path, tf.constant(False)))
        img_list_dataset = tf.data.Dataset.from_tensor_slices(img_list)
        all_dataset = tf.data.Dataset.zip((img_list_dataset, img_dataset))
        all_dataset = all_dataset.batch(spec.batch_size).prefetch(spec.batch_size*4)
        iterator = all_dataset.make_initializable_iterator()
        return iterator, spec.batch_size


def _activation_summary(act):
    """Helper to create summaries for activations.
    Creates a summary that provides a histogram of activations.
    Creates a summary that measure the sparsity of activations.
    Args:
        act: Tensor of activation.
    Returns:
        Nothing.
    """

    tf.summary.histogram(act.op.name + '/activations', act)
    tf.summary.scalar(act.op.name + '/sparsity', tf.nn.zero_fraction(act))


def _activation_summaries(endpoints):
    with tf.name_scope('summaries'):  # pylint: disable=not-context-manager
        for act in endpoints.values():
            _activation_summary(act)


def _rmac(feat_map, rmac_step, reduce_method, deploy):
    """Extract regional vector from the raw feature map.
    The overlap ratio of neighboring regions is 0.5
    Args:
        feat_map: input raw feature map.
        step: the length of the output feature map.
        e.g., step = 3 gives 3 x 3 = 9 regional vectors.
    Returns:
        all_rvec: regional vectors.
    """
    all_rvec = None
    fully_defined = False
    if feat_map.get_shape().is_fully_defined() or deploy:
        batch_size, feat_h, feat_w, feat_dim = feat_map.get_shape().as_list()
        fully_defined = True
    else:
        batch_size, feat_h, feat_w, feat_dim = tf.unstack(tf.shape(feat_map))

    for step in rmac_step:
        if step > 1:
            k_h = (feat_h / (step + 1)) * 2
            s_h = (feat_h - k_h) / (step - 1)
            k_w = (feat_w / (step + 1)) * 2
            s_w = (feat_w - k_w) / (step - 1)
        else:
            # reduce all.
            k_h = feat_h
            s_h = 1
            k_w = feat_w
            s_w = 1

        if fully_defined and (k_h < 1 or k_w < 1):
            # skip the step if the kernal size is smaller that 1.
            continue

        if reduce_method == 'AVG':
            rvec = gen_nn_ops._avg_pool(feat_map, [1, k_h, k_w, 1], [1, s_h, s_w, 1], 'VALID')
        elif reduce_method == 'L2':
            rvec = tf.sqrt(gen_nn_ops._avg_pool(tf.square(feat_map), [
                - 1, k_h, k_w, 1], [1, s_h, s_w, 1], 'VALID'))
        elif reduce_method == 'MAX':
            #rvec = gen_nn_ops._max_pool_v2(feat_map, [1, k_h, k_w, 1], [1, s_h, s_w, 1], 'VALID')
            rvec = tf.nn.max_pool(feat_map, [1, k_h, k_w, 1], [1, s_h, s_w, 1], 'VALID')
        else:
            print(Notify.FAIL, 'Known reduce method:', reduce_method, Notify.ENDC)

        rvec = tf.reshape(rvec, [tf.shape(feat_map)[0], -1, feat_dim])
        rvec = tf.transpose(rvec, [0, 2, 1])
        if all_rvec is None:
            all_rvec = rvec
        else:
            all_rvec = tf.concat([all_rvec, rvec], axis=2)
    return all_rvec


def split_train_and_validation(image_list, label_list, batch_size, kfold):
    """
    split the training data, and select the first 1/kfold samples as validation
    :param image_list: the input image list
    :param label_list: the input label list
    :param batch_size: batch size of networks, used for guaranteeing that validation set is
    multiples of batch_size, to facilitate testing
    :param kfold: kfold in cross validation
    :return: train_list, train_label, validation_list, validation_label
    """
    total_image_num = len(image_list)
    assert total_image_num == len(label_list)
    validation_image_num = total_image_num / kfold
    # make sure validation is multiples of batch_size to facilitate validation
    validation_image_num = (validation_image_num // batch_size) * batch_size
    # split the image list into training and validation
    validation_image_list = image_list[:validation_image_num]
    validation_label_list = label_list[:validation_image_num]
    train_image_list = image_list[validation_image_num:]
    train_label_list = label_list[validation_image_num:]
    assert len(train_image_list) == len(train_label_list)
    assert len(validation_image_list) == len(validation_label_list)
    return train_image_list, train_label_list, validation_image_list, validation_label_list
