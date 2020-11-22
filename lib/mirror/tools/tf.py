#!/usr/bin/env python
"""
Copyright 2018, Tianwei Shen, HKUST.
Copyright 2017, Zixin Luo, HKUST.
TensorFlow tools.
"""

from __future__ import print_function

import os
from math import sqrt

import subprocess

import tensorflow as tf
from tensorflow.contrib.tensorboard.plugins import projector


def put_kernels_on_grid(kernel, display_size=16, pad=1):
    """Visualize conv. filters as an image (mostly for the 1st layer).
    Arranges filters into a grid, with some paddings between adjacent filters.
    Args:
        kernel: Tensor of shape [Y, X, NumChannels, NumKernels]
        pad: Number of black pixels around each filter (between them)
    Return:
        Tensor of shape [(Y+2*pad)*grid_Y, (X+2*pad)*grid_X, NumChannels, 1].
    """

    # get shape of the grid. NumKernels == grid_Y * grid_X
    def factorization(n_kernel):
        """factorization"""
        for i in range(int(sqrt(float(n_kernel))), 0, -1):
            if n_kernel % i == 0:
                if i == 1:
                    print('Who would enter a prime number of filters')
                return (i, int(n_kernel / i))
    (grid_y, grid_x) = factorization(kernel.get_shape()[3].value)
    x_min = tf.reduce_min(kernel)
    x_max = tf.reduce_max(kernel)
    # normalize filters to [0, 1].
    norm_kernel = (kernel - x_min) / (x_max - x_min)
    # put NumKernels to the 1st dimension.
    norm_kernel = tf.transpose(norm_kernel, (3, 0, 1, 2))
    norm_kernel = tf.image.resize_images(norm_kernel, (display_size, display_size))
    # pad X and Y
    x_1 = tf.pad(norm_kernel, tf.constant(
        [[0, 0], [pad, pad], [pad, pad], [0, 0]]), mode='CONSTANT')
    # X and Y dimensions, w.r.t. padding
    y_dim = norm_kernel.get_shape()[1] + 2 * pad
    x_dim = norm_kernel.get_shape()[2] + 2 * pad
    channels = norm_kernel.get_shape()[3]
    # organize grid on Y axis
    x_2 = tf.reshape(x_1, tf.stack([grid_x, y_dim * grid_y, x_dim, channels]))
    # switch X and Y axes
    x_3 = tf.transpose(x_2, (0, 2, 1, 3))
    # organize grid on X axis
    x_4 = tf.reshape(x_3, tf.stack([1, x_dim * grid_x, y_dim * grid_y, channels]))
    # back to normal order (not combining with the next step for clarity)
    x_5 = tf.transpose(x_4, (2, 1, 3, 0))
    # to tf.image_summary order [batch_size, height, width, channels],
    # where in this case batch_size == 1
    x_6 = tf.transpose(x_5, (3, 0, 1, 2))
    # scaling to [0, 255] is not necessary for tensorboard
    return x_6


def tensorboard_projector_visualize(sess, all_feat, log_dir, sprite_image=None, image_width=-1, image_height=-1):
    """
    generate necessary model data and config files for tensorboard projector
    :param sess: the current computing session
    :param all_feat: a 2d numpy feature vector of shape [num_features, feature_dimension]
    :param log_dir: the output log folder
    :param sprite_image: (optional) the big image in a row-major fashion for visualization
    :param image_width: (optional) the width for a single data point in sprite image
    :param image_height: (optional) the height for a single data point in sprite image
    :return: None
    """

    # create embedding summary and run it
    summary_writer = tf.summary.FileWriter(log_dir)
    embedding_var = tf.Variable(all_feat, name='feature_embedding')
    sess.run(embedding_var.initializer)

    # create projector config
    config = projector.ProjectorConfig()
    embedding = config.embeddings.add()
    embedding.tensor_name = embedding_var.name

    # adding sprite images
    if sprite_image != None and image_width != -1 and image_height != -1:
        embedding.sprite.image_path = sprite_image
        embedding.sprite.single_image_dim.extend([image_width, image_height])

    projector.visualize_embeddings(summary_writer, config)

    # save the embedding
    saver_embed = tf.train.Saver([embedding_var])
    saver_embed.save(sess, os.path.join(log_dir, 'embedding_test.ckpt'))


def freeze_model(model_dir, model_name, tf_pytool, ckpt_file, input_nodes, output_nodes, opt=True):
    """
    Args:
        model_dir: dir to save the model.
        model_name: name of the model.
        tf_pytool: path to TensorFlow python tools, e.g., <tf_root>/tensorflow/python/tools
        ckpt_file: model params stored in ckpt file.
        input_nodes: input node names.
        output_nodes: output node names.
    Returns:
        Nothing.
    """

    input_graph = os.path.join(model_dir, model_name) + '.pbtxt'
    output_graph = os.path.join(model_dir, model_name) + '.pb'

    # write the plain proto text of graph definition.
    config = tf.ConfigProto()
    config.gpu_options.allow_growth = True
    with tf.Session(config=config) as sess:
        tf.train.write_graph(sess.graph_def, model_dir, model_name + '.pbtxt', as_text=True)
    
    # freeze params into a single binary file (proto buffer).
    subprocess.call(['python', os.path.join(tf_pytool, 'freeze_graph.py'),
                     '--input_graph', input_graph,
                     '--input_checkpoint', ckpt_file,
                     '--output_node_names', output_nodes if type(
                         output_nodes) is str or unicode else ','.join(output_nodes),
                     '--input_binary', 'False',
                     '--output_graph', output_graph])

    if opt:
        # optimize the frozen graph to improve inference performance.
        subprocess.call(['python', os.path.join(tf_pytool, 'optimize_for_inference.py'),
                         '--input', output_graph,
                         '--output', output_graph,
                         '--frozen_graph', 'True',
                         '--input_names', input_nodes if type(
                             input_nodes) is str or unicode else ','.join(input_nodes),
                         '--output_names', output_nodes if type(output_nodes) is str or unicode else ','.join(output_nodes)])


def load_frozen_model(pb_path, prefix='', print_nodes=False):
    """Load frozen model (.pb file) for testing.
    After restoring the model, you can access the operator by
    graph.get_tensor_by_name('<prefix>/<op_name>')
    Args:
        pb_path: the path of frozen model.
        prefix: prefix added to the operator name.
        print_nodes: whether to print node names in the terminal.
    Returns:
        graph: tensorflow graph definition.
    """
    if os.path.exists(pb_path):
        with tf.gfile.GFile(pb_path, "rb") as f:
            graph_def = tf.GraphDef()
            graph_def.ParseFromString(f.read())

        with tf.Graph().as_default() as graph:
            tf.import_graph_def(
                graph_def,
                name=prefix
            )
            if print_nodes:
                for op in graph.get_operations():
                    print(op.name)
            return graph
    else:
        print('Model file does not exist', pb_path)
        exit(-1)


def average_gradients(tower_grads):
    """Calculate the average gradient for each shared variable across all towers.
    Note that this function provides a synchronization point across all towers.
    Args:
    tower_grads: List of lists of (gradient, variable) tuples. The outer list
        is over individual gradients. The inner list is over the gradient
        calculation for each tower.
    Returns:
        List of pairs of (gradient, variable) where the gradient has been averaged
        across all towers.
    """
    average_grads = []
    for grad_and_vars in zip(*tower_grads):
        # Note that each grad_and_vars looks like the following:
        #   ((grad0_gpu0, var0_gpu0), ... , (grad0_gpuN, var0_gpuN))
        grads = []
        for g, _ in grad_and_vars:
            # Add 0 dimension to the gradients to represent the tower.
            expanded_g = tf.expand_dims(g, 0)

            # Append on a 'tower' dimension which we will average over below.
            grads.append(expanded_g)

        # Average over the 'tower' dimension.
        grad = tf.concat(axis=0, values=grads)
        grad = tf.reduce_mean(grad, 0)

        # Keep in mind that the Variables are redundant because they are shared
        # across towers. So .. we will just return the first tower's pointer to
        # the Variable.
        v = grad_and_vars[0][1]
        grad_and_var = (grad, v)
        average_grads.append(grad_and_var)
    return average_grads


