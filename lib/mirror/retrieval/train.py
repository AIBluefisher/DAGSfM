#!/usr/bin/env python
"""
Copyright 2017, Tianwei Shen, HKUST.
Copyright 2017, Zixin Luo, HKUST.
Training geometric retrieval network
"""

from __future__ import print_function

import os
import time
import sys
import struct
import numpy as np
import tensorflow as tf

from model import training, get_triplet_input_tensors, recover_from_pretrained

CURDIR = os.path.dirname(__file__)
sys.path.append(os.path.abspath(os.path.join(CURDIR, '.')))
sys.path.append(os.path.abspath(os.path.join(CURDIR, '..')))

import tools.tf
from tools.common import Notify, read_list

FLAGS = tf.app.flags.FLAGS

# Params for config.
tf.app.flags.DEFINE_integer('ckpt_step', None, """ckpt step.""")
tf.app.flags.DEFINE_string('np_model_path', '', """numpy model parameter path""")
tf.app.flags.DEFINE_string('log_dir', './tf_log', """Path to store the log.""")
tf.app.flags.DEFINE_string('save_dir', './ckpt', """Path to save the model.""")
tf.app.flags.DEFINE_string('gl3d', None, """Path to gl3d""")

tf.app.flags.DEFINE_boolean('is_training', True, """Flag to training model""")
tf.app.flags.DEFINE_boolean('with_hard', False, """With inner loss.""")
tf.app.flags.DEFINE_boolean('with_weak', False, """With weak positive loss.""")
tf.app.flags.DEFINE_integer('test_step', 1000, "the every number of steps to run test")
tf.app.flags.DEFINE_integer('num_gpus', 1, "the number of gpus in a machine")
tf.app.flags.DEFINE_string('net', 'half_googlenet', """network used, half_googlenet or googlenet""")

# Params for solver.
tf.app.flags.DEFINE_float('base_lr', 0.002,
                          """Base learning rate.""")
tf.app.flags.DEFINE_integer('max_steps', 1000000,
                            """Max training iteration.""")
tf.app.flags.DEFINE_integer('display', 25,
                            """Interval of loginfo display.""")
tf.app.flags.DEFINE_integer('stepvalue', 10000,
                            """Step interval to decay learning rate.""")
tf.app.flags.DEFINE_integer('snapshot', 1000,
                            """Step interval to save the model.""")
tf.app.flags.DEFINE_float('gamma', 0.9,
                          """Learning rate decay rate.""")
tf.app.flags.DEFINE_float('momentum', 0.9,
                          """momentum in when using SGD with momentum solver""")
tf.app.flags.DEFINE_float('weight_decay', 0.0001,
                          """Fraction of regularization term.""")


def get_refine_layers_name(net_type):
    new_layers = []
    if net_type == 'googlenet':
        new_layers = ['inception_4a_loss', 'inception_4d_loss', 'inception_5b_loss']
    elif net_type == 'half_googlenet':
        new_layers = ['inception_4a_output', 'inception_4d_output', 'inception_5b_output']
    elif net_type == 'vgg':
        new_layers = ['conv5_3']
    elif net_type.find('resnet') != -1:
        new_layers = ['res5c_branch2c']
    else:
        print(Notify.FAIL, 'network type not supported')
        exit(-1)
    return new_layers


def solver(loss, accuracy, new_layers):
    """Solver."""
    # Get weight variable list.
    weights_list = tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES)

    # Separate the weights, we want the new layer to have a higher learning rate
    finetuned_layer_vars = []
    new_layers_vars = []
    for var in weights_list:
        for layer_name in new_layers:
            if var.name.find(layer_name) != -1:
                new_layers_vars.append(var)
            else:
                finetuned_layer_vars.append(var)
    # new layers is not empty when the loss layer is trainable, but will be empty when the loss layer is concat
    print(Notify.INFO, 'new layers:', new_layers_vars, Notify.ENDC)

    # Apply regularization to variables.
    reg_loss = tf.contrib.layers.apply_regularization(
        tf.contrib.layers.l2_regularizer(scale=FLAGS.weight_decay), weights_list)
    # Get global step counter.
    global_step = tf.Variable(0, trainable=False, name='global_step')
    # Decay the learning rate exponentially based on the number of steps.
    lr_op = tf.train.exponential_decay(learning_rate=FLAGS.base_lr,
                                       global_step=global_step,
                                       decay_steps=FLAGS.stepvalue,
                                       decay_rate=FLAGS.gamma,
                                       name='lr')

    tf.summary.scalar('lr', lr_op)
    loss_layer_num = len(loss)
    final_loss = reg_loss
    # easy loss
    for i in range(loss_layer_num):
        final_loss = final_loss + loss[i][0]
        tf.summary.scalar('EASY_loss'+str(i), loss[i][0])
        tf.summary.scalar('accuracy'+str(i), accuracy[i][0])
    next_loss = 1

    # weak loss
    if FLAGS.with_weak:
        for i in range(loss_layer_num):
            final_loss = final_loss + loss[i][next_loss]
            tf.summary.scalar('weak_loss'+str(i), loss[i][next_loss])
            tf.summary.scalar('weak_accuracy'+str(i), accuracy[i][next_loss])
        next_loss = next_loss+1

    # hard loss
    if FLAGS.with_hard:
        for i in range(loss_layer_num):
            final_loss = final_loss + loss[i][next_loss]
            tf.summary.scalar('hard_loss'+str(i), loss[i][next_loss])
            tf.summary.scalar('hard_accuracy'+str(i), accuracy[i][next_loss])

    # For networks with batch normalization layers, it is necessary to
    # explicitly fetch its moving statistics and add them to the optimizer.
    bn_list = tf.get_collection(tf.GraphKeys.UPDATE_OPS)
    with tf.control_dependencies(bn_list):
        # Get the optimizer. Moving statistics are added to optimizer.
        #opt = tf.train.AdamOptimizer(learning_rate=lr_op).minimize(final_loss, global_step=global_step)
        grads = tf.gradients(final_loss, finetuned_layer_vars + new_layers_vars)
        grads1 = grads[:len(finetuned_layer_vars)]
        grads2 = grads[len(finetuned_layer_vars):]

        # finetuned layers
        opt = tf.train.MomentumOptimizer(learning_rate=lr_op, momentum=FLAGS.momentum)
        train_op = opt.apply_gradients(zip(grads1, finetuned_layer_vars), global_step=global_step)

        # new layers, if there is any
        if grads2 != []:
            opt_new = tf.train.MomentumOptimizer(learning_rate=10*lr_op, momentum=FLAGS.momentum)
            train_op2 = opt_new.apply_gradients(zip(grads2, new_layers_vars), global_step=global_step)
            train_op = tf.group(train_op, train_op2) # group the two ops

    for var in weights_list:
        tf.summary.histogram(var.op.name, var)
    return train_op, final_loss, lr_op


def train(train_list, image_list, net_type):
    config = tf.ConfigProto()
    config.gpu_options.allow_growth = True
    sess = tf.Session(config=config)

    # read buffered training and testing input tensors
    batch_sample = get_triplet_input_tensors(os.path.join(FLAGS.gl3d, ''), train_list, image_list, sess, net_type)
    # Construct training networks.
    new_layers = get_refine_layers_name(net_type)
    feature_tower, loss, accuracy = training(batch_sample.get_next(), net_type, new_layers)

    # Set up solver for triplet loss
    if FLAGS.is_training:
        # Construct the solver.
        opt, total_loss, lr = solver(loss, accuracy, new_layers)
        # Create a saver.
        saver = tf.train.Saver(tf.global_variables(), max_to_keep=None)

        # Visualize first convolution filters.
        if net_type == 'googlenet' or net_type == 'half_googlenet':
            first_layer_name = 'conv1_7x7_s2'
        elif net_type == 'vgg':
            first_layer_name = 'conv1_1'
        elif net_type.find('resnet') != -1:
            first_layer_name = 'conv1'
        else:
            first_layer_name = None
        if first_layer_name != None:
            with tf.variable_scope(first_layer_name):  # pylint: disable=not-context-manager
                tf.get_variable_scope().reuse_variables()
                weights = tf.get_variable('kernel')
                grid = tools.tf.put_kernels_on_grid(weights)
                tf.summary.image('features', grid, max_outputs=1)

        # Build the summary operation.
        summary_op = tf.summary.merge_all()

        init_op = tf.global_variables_initializer()
        sess.run(init_op)
        # recover from ckpt model or numpy model, if there is any.
        checkpoint_path = os.path.join(FLAGS.save_dir, 'model.ckpt')
        if FLAGS.ckpt_step != None or FLAGS.np_model_path != None:
            step = recover_from_pretrained(sess, feature_tower, checkpoint_path,
                                           FLAGS.ckpt_step, FLAGS.np_model_path)

        # Create summary writer.
        if FLAGS.is_training:
            summary_writer = tf.summary.FileWriter(
                FLAGS.log_dir, None, filename_suffix=net_type)

        # output the number of parameters
        with tf.name_scope('parameter_count'):
            param_count_op = tf.reduce_sum([tf.reduce_prod(tf.shape(v)) for v in tf.trainable_variables()])
        print(Notify.INFO, 'parameter count =', sess.run(param_count_op), Notify.ENDC)

        while step <= FLAGS.max_steps:
            start_time = time.time()
            if FLAGS.is_training:
                summary_str, _, total_loss_val, lr_val, loss_val, accuracy_val  = \
                    sess.run([summary_op, opt, total_loss, lr, loss, accuracy])
            duration = time.time() - start_time

            if step % FLAGS.display == 0 or not FLAGS.is_training:
                sum_str = ('lr = %.8f, total_loss = %.4f')

                format_str = ('step %d, EASY loss in stage %d = %.8f, accuracy = %.4f (%.3f sec/step)')
                for z in range(len(loss_val)):
                    print(Notify.INFO, format_str %
                          (step, z, loss_val[z][0], accuracy_val[z][0], duration), Notify.ENDC)
                
                next_loss = 1
                if FLAGS.with_weak:
                    format_str = ('step %d, WEAK loss in stage %d = %.8f, accuracy = %.4f (%.3f sec/step)')
                    for z in range(len(loss_val)):
                        print(Notify.INFO, format_str %
                              (step, z, loss_val[z][next_loss], accuracy_val[z][next_loss], duration), Notify.ENDC)
                    next_loss = next_loss+1

                if FLAGS.with_hard:
                    format_str = ('step %d, HARD loss in stage %d = %.8f, accuracy = %.4f (%.3f sec/step)')
                    for z in range(len(loss_val)):
                        print(Notify.INFO, format_str %
                              (step, z, loss_val[z][next_loss], accuracy_val[z][next_loss], duration), Notify.ENDC)
                print(Notify.INFO, sum_str % (lr_val, total_loss_val), Notify.ENDC)

            # Write summary.
            if step % FLAGS.display == 0 and FLAGS.is_training:
                summary_writer.add_summary(summary_str, step)

            # Save the model checkpoint periodically.
            step += 1
            if (step % FLAGS.snapshot == 0 or step == FLAGS.max_steps) and FLAGS.is_training:
                saver.save(sess, checkpoint_path, global_step=step)


def read_samples(sample_path):
    """Read samples."""
    all_samples = []
    training_file = open(sample_path, 'rb')

    data = training_file.read(4)    # 4 is the size of a integer
    dataset_num = struct.unpack('i', data)
    print(Notify.INFO, '# Dataset', dataset_num[0], Notify.ENDC)
    for _ in range(dataset_num[0]):
        record = []
        data = training_file.read(4)
        image_num = struct.unpack('i', data)
        for _ in range(image_num[0]):
            data = training_file.read(8)
            ref_idx, pos_num = struct.unpack('2i', data)
            data = training_file.read(4 * pos_num)
            pos_idx = struct.unpack('i' * pos_num, data)
            data = training_file.read(4)
            neg_num = struct.unpack('i', data)
            data = training_file.read(4 * neg_num[0])
            neg_idx = struct.unpack('i' * neg_num[0], data)

            all_mask = []
            for _ in range(pos_num):
                data = training_file.read(392)
                mask = np.array(struct.unpack('?' * 392, data))
                all_mask.append(mask)
            record.append([ref_idx, pos_idx, neg_idx, all_mask])
        all_samples.append(record)
    return all_samples


def main(argv=None):  # pylint: disable=unused-argument
    # read image list (train and test)
    image_list_path = os.path.join(FLAGS.gl3d, 'list', 'global_stats', 'global_img_list.txt')
    train_list_path = os.path.join(FLAGS.gl3d, 'forward', 'train_list_with_mask.bin')

    image_list = read_list(image_list_path)
    train_list = read_samples(train_list_path)
    train(train_list, image_list, FLAGS.net)


if __name__ == '__main__':
    tf.flags.mark_flags_as_required(['gl3d'])
    tf.app.run()
