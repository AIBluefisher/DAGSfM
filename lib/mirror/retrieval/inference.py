#!/usr/bin/env python
"""
Copyright 2017, Zixin Luo, HKUST.
Copyright 2017, Tianwei Shen, HKUST.
Extract deep features for image retrieval.

Usage:
    (Required) img_list is in format {ABSOLUTE_PATH1\nABSOLUTE_PATH2\n...}
    (Optional) img_size specifies the fixed input image size. Leaving it to None
    allows arbitrary input size.
    (Required) model_data is a numpy file that will be loaded into networks.
    (Optional) output_folder specifies the folder that features store in. Leave
    it to None then feature files store aside of image files.
"""

from __future__ import print_function

import sys
import os
import glob
import time
import thread
from threading import Thread
from Queue import Queue
from math import floor, ceil

from scipy.misc import imread, imsave
import progressbar
import numpy as np
import tensorflow as tf

_file_path = os.path.abspath(sys.argv[0])
_cur_dir = os.path.dirname(_file_path)
sys.path.append(os.path.dirname(_cur_dir))

from model import inference, recover_from_pretrained
from tools.common import read_list, Notify

FLAGS = tf.app.flags.FLAGS
tf.app.flags.DEFINE_string('gl3d_root', 'None',
                           """Path to GL3D dataset.""")
tf.app.flags.DEFINE_string('ckpt_path', './ckpt/model.ckpt',
                           """Path to restore the model.""")
tf.app.flags.DEFINE_integer('ckpt_step', None, """ckpt step.""")
tf.app.flags.DEFINE_string('img_list', None, """Path to image list.""")
tf.app.flags.DEFINE_integer('img_size', 224, """Input image size.""")
tf.app.flags.DEFINE_string('model_data', None, """Path to model data.""")
tf.app.flags.DEFINE_string('output_folder', None, """Path to output folder.""")
tf.app.flags.DEFINE_string('pool', None, """pooling method: L2, Max, AVE""")
tf.app.flags.DEFINE_string('net', 'googlenet',
                           """network tyep: googlenet or half_googlenet""")
tf.app.flags.DEFINE_string('rmac_step', None, """RMAC steps. e.g., 1,3,5 for three scales,
                            1 for single scale (equals to MAC)""")
tf.app.flags.DEFINE_string('oxford_gt', None, """extract oxford query""")
tf.app.flags.DEFINE_string('log_dir', None, """Path to store the log.""")
tf.app.flags.DEFINE_string('save_suffix', '.npy', """save suffix.""")
tf.app.flags.DEFINE_integer('num_gpus', 1, "the number of gpus in a machine")
tf.app.flags.DEFINE_boolean('multi_scale', False, "Whether to enable multi-scale solution")


def oxford_query_images(groundtruth_dir, image_list_path, dataset, cropped=False):
    with open(image_list_path, 'r') as f:
        first_line = f.readline()
        image_dir = os.path.split(first_line)[0]
    img_list = []
    query_img_out_dir = './oxford_queries'
    if not os.path.exists(query_img_out_dir):
        os.mkdir(query_img_out_dir)
    for f in glob.iglob(os.path.join(groundtruth_dir, '*_query.txt')):
        query_name = os.path.splitext(os.path.basename(f))[
            0].replace('_query', '')
        img_name, x, y, w, h = open(f).read().strip().split(' ')

        if dataset == 'oxford':
            img_name = img_name.replace('oxc1_', '')
        image_path = os.path.join(image_dir, '%s.jpg' % img_name)
        img = imread(image_path)

        if cropped:
            (x, y, w, h) = map(float, (x, y, w, h))
            img = img[int(floor(y)): int(floor(y + h)), int(floor(x)): int(floor(x + w))]
        output_name = os.path.join(query_img_out_dir, query_name) + '.jpg'
        imsave(output_name, img)
        img_list.append(output_name)

    return img_list


def get_output_tensor_name(net_type):
    if net_type == 'half_googlenet' or net_type == 'googlenet' or net_type == 'googlenet_bn':
        return 'inception_5b_output'
    elif net_type == 'googlenet_places':
        return 'icp9_out'
    elif net_type == 'vgg' or net_type.find('resnet') != -1:
        return 'pool5'
    else:
        print(Notify.FAIL, 'network type not supported')
        exit(-1)


def extract_features(img_list, ckpt_path, ckpt_step, net_type, rmac_step, pool='MAC', foutput_folder=None, device_idx=0):
    tf.reset_default_graph()
    tf.set_random_seed(1)
    # create net with image inputs
    print(Notify.INFO, 'img_size is', FLAGS.img_size, Notify.ENDC)
    config = tf.ConfigProto(allow_soft_placement=True)
    config.gpu_options.allow_growth = True
    config.graph_options.optimizer_options.global_jit_level = tf.OptimizerOptions.ON_1
    with tf.Session(config=config) as sess:
        output_tensor_name = get_output_tensor_name(net_type)
        with tf.device('/gpu:%d' % device_idx):
            net, feat_op, batch_size, image_filenames = inference(sess, img_list, net_type,
                                                                  rmac_step, output_tensor_name,
                                                                  reduce_method=pool)
        # load model parameters.
        # see https://github.com/tensorflow/tensorflow/issues/1355
        sess.run(tf.local_variables_initializer())
        recover_from_pretrained(sess, net, ckpt_path, ckpt_step, FLAGS.model_data)

        # multi-thread starts.
        coord = tf.train.Coordinator()
        threads = tf.train.start_queue_runners(coord=coord)
        # infer feature.
        print(Notify.INFO, 'Network type', net_type, '(output tensor:', output_tensor_name, ')', Notify.ENDC)
        print(Notify.INFO, 'Feature extraction using', pool, Notify.ENDC)
        print(Notify.INFO, '# Image', len(img_list), Notify.ENDC)
        print(Notify.INFO, 'Batch size', batch_size, Notify.ENDC)
        num_loop = int(ceil(float(len(img_list)) / float(batch_size)))
        all_feat = []
        all_filenames = []

        prog_bar = progressbar.ProgressBar()
        prog_bar.maxval = num_loop

        def _worker(q, all_feat, all_filenames):
            while True:
                feat, batch_filenames = q.get()
                if feat is None or batch_filenames is None:
                    return
                all_feat.append(feat)
                all_filenames.append(batch_filenames)
                q.task_done()

        q = Queue()
        worker_thread = Thread(target=_worker, args=(q, all_feat, all_filenames))
        worker_thread.daemon = True
        worker_thread.start()

        prog_bar.start()
        for i in range(num_loop):
            feat, batch_filenames = sess.run([feat_op, image_filenames])
            q.put([feat, batch_filenames])
            prog_bar.update(i)
        # poison pill
        q.put([None, None])

        worker_thread.join()

        # multi-thread stops.
        coord.request_stop()
        coord.join(threads)

        # save each feature to different .npy files
        batch_size = all_feat[i].shape[0]
        for i in range(len(all_feat)):
            for j in range(batch_size):
                output_folder = foutput_folder
                if output_folder is None:
                    if batch_size == 1:
                        output_folder = os.path.split(all_filenames[i])[0]
                    else:
                        output_folder = os.path.split(all_filenames[i][j])[0]
                elif not os.path.exists(output_folder):
                    os.mkdir(output_folder)
                if batch_size == 1:
                    np_filename = os.path.splitext(os.path.basename(all_filenames[i]))[0] + FLAGS.save_suffix
                else:
                    np_filename = os.path.splitext(
                        os.path.basename(all_filenames[i][j]))[0] + FLAGS.save_suffix
                save_path = os.path.join(output_folder, np_filename)
                np.save(save_path, all_feat[i][j])


def extract_feature_thread(img_list, batch_num, batch_size, rmac_step, device_idx, lock):
    for i in range(batch_num):
        if i % FLAGS.num_gpus != device_idx:
            continue
        print(Notify.INFO, 'Iteration:', i, Notify.ENDC)
        if i != batch_num-1:
            sub_img_list = img_list[i*batch_size:(i+1)*batch_size]
        else:
            sub_img_list = img_list[i*batch_size:]

        if len(sub_img_list) > 0:
            extract_features(sub_img_list, FLAGS.ckpt_path,
                             FLAGS.ckpt_step, FLAGS.net, rmac_step, FLAGS.pool, FLAGS.output_folder, device_idx)
    lock.release()


def main(argv=None):  # pylint: disable=unused-argument
    """Program entrance."""
    if FLAGS.oxford_gt is None:
        img_list = read_list(FLAGS.img_list)
        if FLAGS.gl3d_root != None:
            img_list = [os.path.join(FLAGS.gl3d_root, p) for p in img_list]
    else:
        img_list = oxford_query_images(FLAGS.oxford_gt, FLAGS.img_list, 'oxford', False)
    rmac_step = map(int, FLAGS.rmac_step.split(','))
    if FLAGS.img_size == 224:
        rmac_step = [1]
    img_list_len = len(img_list)
    batch_size = 10000
    batch_num = int(ceil(float(img_list_len) / float(batch_size)))
    if batch_num > 1:
        print(Notify.INFO, 'Image list of size', img_list_len, 'is too long, split to', batch_num, 'batches', Notify.ENDC)

    locks = []
    for i in range(FLAGS.num_gpus):
        lock = thread.allocate_lock()
        lock.acquire()
        locks.append(lock)

    for i in range(FLAGS.num_gpus):
        thread.start_new_thread(extract_feature_thread, (img_list, batch_num, batch_size, rmac_step, i, locks[i]))

    for lock in locks:
        while lock.locked():
            time.sleep(1)
            pass


if __name__ == '__main__':
    tf.flags.mark_flags_as_required(['rmac_step', 'img_list'])
    tf.app.run()
